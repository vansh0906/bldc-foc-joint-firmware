/* Joint bus: half-duplex RS-485, multi-drop, little-endian binary frames.
 *
 *   AA 55 | id | len | instr | data (len-1 bytes) | CRC-16/MODBUS (lo, hi) over id..data
 *
 * A host runs a 1 kHz cycle with one broadcast SYNC_CMD frame carrying a command for every
 * joint; each joint answers with its status in its own time slot (its position in the list),
 * so there are no collisions and no polling. Frames are resynchronized on the AA 55 header,
 * on an idle gap, and rejected on a length or CRC error. */
#include <string.h>

#include "joint.h"

/* instructions */
#define I_PING      0x01u
#define I_READ      0x02u
#define I_WRITE     0x03u
#define I_CMD       0x11u   /* one joint: same payload as a SYNC_CMD entry without the id */
#define I_SYNC_CMD  0x10u   /* broadcast: n entries of 16 bytes */
#define I_ENABLE    0x20u
#define I_DISABLE   0x21u
#define I_CLEAR     0x22u
#define I_CALIBRATE 0x23u
#define I_SAVE      0x24u
#define I_REPLY     0x80u

#define STATUS_LEN  17u
#define REPLY_BYTES (2u + 1u + 1u + 1u + STATUS_LEN + 2u)
#define ENTRY_LEN   16u
#define NREG        27u

enum { P_SYNC1, P_SYNC2, P_ID, P_LEN, P_DATA, P_CRC1, P_CRC2 };

uint16_t crc16_modbus(const uint8_t *d, uint16_t n)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    }
    return crc;
}

void jbus_reset(jbus_t *b)
{
    memset(b, 0, sizeof *b);
}

/* RX interrupt: queue the byte; an idle gap longer than a few byte times is queued as a marker */
void app_uart_rx(app_t *a, uint8_t byte, uint32_t t_us)
{
    jbus_t *b = &a->bus;
    uint32_t byte_us = 10000000u / a->prm.baud + 1u;
    uint32_t gap = 4u * byte_us > 10u ? 4u * byte_us : 10u;
    uint16_t next = (uint16_t)((b->head + 1u) % JBUS_RING);
    if ((uint32_t)(t_us - b->last_rx_us) > gap && next != b->tail) {
        b->ring[b->head] = 0x100u;
        b->ring_t[b->head] = t_us;
        b->head = next;
        next = (uint16_t)((b->head + 1u) % JBUS_RING);
    }
    b->last_rx_us = t_us;
    if (next == b->tail) { b->overflow++; return; }
    b->ring[b->head] = byte;
    b->ring_t[b->head] = t_us;
    b->head = next;
}

static void put_f32(uint8_t *p, float v) { memcpy(p, &v, 4); }
static float get_f32(const uint8_t *p) { float v; memcpy(&v, p, 4); return v; }
static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void put_u32(uint8_t *p, uint32_t v) { for (int k = 0; k < 4; k++) p[k] = (uint8_t)(v >> (8 * k)); }
static uint32_t get_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static uint16_t frame(uint8_t *out, uint8_t id, uint8_t instr, const uint8_t *data, uint8_t n)
{
    out[0] = 0xAA; out[1] = 0x55; out[2] = id; out[3] = (uint8_t)(n + 1u); out[4] = instr;
    if (n) memcpy(&out[5], data, n);
    uint16_t crc = crc16_modbus(&out[2], (uint16_t)(3u + n));
    put_u16(&out[5 + n], crc);
    return (uint16_t)(7u + n);
}

uint16_t jbus_build_status(app_t *a, uint8_t instr, uint8_t *out)
{
    uint8_t d[STATUS_LEN];
    float iq = a->foc.iq * 1000.0f;
    float vb = a->prot.vbus_f * 100.0f;
    float tw = a->prot.t_wind_est;
    d[0] = (uint8_t)a->state;
    d[1] = (uint8_t)a->ctrl.mode;
    put_u16(&d[2], a->faults);
    put_f32(&d[4], a->enc.pos);
    put_f32(&d[8], a->enc.vel);
    put_u16(&d[12], (uint16_t)(int16_t)clampf(iq, -32767.0f, 32767.0f));
    put_u16(&d[14], (uint16_t)clampf(vb, 0.0f, 65535.0f));
    d[16] = (uint8_t)(int8_t)clampf(tw, -128.0f, 127.0f);
    return frame(out, a->prm.node_id, (uint8_t)(instr | I_REPLY), d, STATUS_LEN);
}

static void reply_at(app_t *a, uint16_t n, uint32_t t)
{
    a->hal.ops->uart_send(a->hal.ctx, a->bus.tx, n, t);
    a->bus.replies++;
}

static void apply_entry(app_t *a, const uint8_t *e)
{
    /* mode u8 | pos f32 | vel f32 | tau_ff i16 mNm | kp u16 0.01 Nm/rad | kd u16 1e-4 Nm s/rad */
    jmode_t mode = (jmode_t)(e[0] <= MODE_IMPEDANCE ? e[0] : MODE_TORQUE);
    float tau = (float)(int16_t)get_u16(&e[9]) * 0.001f;
    float kp = (float)get_u16(&e[11]) * 0.01f;
    float kd = (float)get_u16(&e[13]) * 1e-4f;
    app_set_command(a, mode, get_f32(&e[1]), get_f32(&e[5]), tau, kp, kd);
}

/* register file: 4 bytes per register */
static bool reg_read(app_t *a, uint8_t r, uint8_t *out)
{
    params_t *p = &a->prm;
    switch (r) {
    case 0: put_u32(out, ((uint32_t)MODEL_ID << 16) | FW_VERSION); return true;
    case 1: put_u32(out, p->node_id); return true;
    case 2: put_f32(out, p->R); return true;
    case 3: put_f32(out, p->L); return true;
    case 4: put_f32(out, p->cur_bw_hz); return true;
    case 5: put_f32(out, p->vel_kp); return true;
    case 6: put_f32(out, p->vel_ki); return true;
    case 7: put_f32(out, p->pos_kp); return true;
    case 8: put_f32(out, p->i_limit); return true;
    case 9: put_f32(out, p->vel_limit); return true;
    case 10: put_f32(out, p->kd_safe); return true;
    case 11: put_u32(out, p->comm_timeout_ms); return true;
    case 12: put_u32(out, p->baud); return true;
    case 13: put_u32(out, p->cal_valid); return true;
    case 14: case 15: case 16: case 17: case 18: case 19: put_f32(out, p->hall_b[r - 14]); return true;
    case 20: put_u32(out, (uint32_t)a->enc.src_override); return true;
    case 21: put_u32(out, a->bus.frames_ok); return true;
    case 22: put_u32(out, a->bus.crc_err); return true;
    case 23: put_f32(out, a->prot.t_wind_est); return true;
    case 24: put_f32(out, a->prot.t_ntc); return true;
    case 25: put_u32(out, (uint32_t)a->enc.src); return true;
    case 26: put_u32(out, a->prot.ntc_only); return true;
    default: return false;
    }
}

static bool reg_write(app_t *a, uint8_t r, const uint8_t *in)
{
    params_t *p = &a->prm;
    float f = get_f32(in);
    uint32_t u = get_u32(in);
    bool running = a->state == ST_RUN || a->state == ST_CAL;
    switch (r) {
    case 1: if (u < 1u || u > 0xFDu || running) return false; p->node_id = (uint8_t)u; return true;
    case 2: if (!(f > 0.01f && f < 5.0f) || running) return false; p->R = f; params_apply_gains(a); return true;
    case 3: if (!(f > 1e-5f && f < 0.01f) || running) return false; p->L = f; params_apply_gains(a); return true;
    case 4: if (!(f >= 100.0f && f <= 3000.0f) || running) return false; p->cur_bw_hz = f; params_apply_gains(a); return true;
    case 5: if (!(f >= 0.0f && f < 1.0f)) return false; p->vel_kp = f; return true;
    case 6: if (!(f >= 0.0f && f < 100.0f)) return false; p->vel_ki = f; return true;
    case 7: if (!(f >= 0.0f && f < 1000.0f)) return false; p->pos_kp = f; return true;
    case 8: if (!(f >= 0.0f && f <= I_MAX)) return false; p->i_limit = f; return true;
    case 9: if (!(f > 0.0f && f <= VEL_LIMIT)) return false; p->vel_limit = f; return true;
    case 10: if (!(f >= 0.0f && f < 1.0f)) return false; p->kd_safe = f; return true;
    case 11: if (u < 5u || u > 5000u) return false; p->comm_timeout_ms = (uint16_t)u; return true;
    case 20: if (u > ANG_SIXSTEP || u == ANG_FORCED) return false; a->enc.src_override = (angsrc_t)u; return true;
    case 26: a->prot.ntc_only = (uint8_t)(u != 0u); return true;
    default: return false;
    }
}

static void dispatch(app_t *a, uint32_t t_end)
{
    jbus_t *b = &a->bus;
    uint8_t id = b->buf[0];
    uint8_t len = b->buf[1];
    uint8_t instr = b->buf[2];
    const uint8_t *d = &b->buf[3];
    uint8_t n = (uint8_t)(len - 1u);
    uint8_t me = a->prm.node_id;
    uint32_t t_reply = t_end + JBUS_TURNAROUND_US;

    if (instr & I_REPLY) return;                 /* another joint's answer */
    if (instr == I_SYNC_CMD) {
        if (id != JBUS_BROADCAST || n < 1u) return;
        uint8_t cnt = d[0];
        if (n != (uint8_t)(1u + cnt * ENTRY_LEN)) { b->len_err++; return; }
        for (uint8_t k = 0; k < cnt; k++) {
            const uint8_t *e = &d[1 + k * ENTRY_LEN];
            if (e[0] != me) continue;
            apply_entry(a, &e[1]);
            uint32_t slot_us = (REPLY_BYTES * 10000000u) / a->prm.baud + 1u + JBUS_GUARD_US;
            uint16_t m = jbus_build_status(a, I_SYNC_CMD, b->tx);
            reply_at(a, m, t_reply + (uint32_t)k * slot_us);
            return;
        }
        return;
    }
    if (id != me && id != JBUS_BROADCAST) return;
    bool answer = id == me;
    bool ok = true;
    switch (instr) {
    case I_PING: break;
    case I_CMD: if (n != ENTRY_LEN - 1u) ok = false; else apply_entry(a, d); break;
    case I_ENABLE: ok = app_enable(a); break;
    case I_DISABLE: app_disable(a); break;
    case I_CLEAR: ok = app_clear_faults(a); break;
    case I_CALIBRATE: ok = app_start_cal(a); break;
    case I_SAVE: ok = a->state != ST_RUN && a->state != ST_CAL && params_save(a); break;
    case I_READ: {
        if (n != 2u || d[1] == 0u || d[1] > 16u) { ok = false; break; }
        uint8_t out[64];
        for (uint8_t k = 0; k < d[1]; k++)
            if (!reg_read(a, (uint8_t)(d[0] + k), &out[4 * k])) { ok = false; break; }
        if (ok && answer) {
            uint16_t m = frame(b->tx, me, (uint8_t)(I_READ | I_REPLY), out, (uint8_t)(4u * d[1]));
            reply_at(a, m, t_reply);
        }
        if (ok) return;
        break;
    }
    case I_WRITE: {
        if (n < 2u || n != (uint8_t)(2u + 4u * d[1])) { ok = false; break; }
        for (uint8_t k = 0; k < d[1]; k++)
            if (!reg_write(a, (uint8_t)(d[0] + k), &d[2 + 4 * k])) ok = false;
        break;
    }
    default: ok = false; break;
    }
    if (!answer) return;
    /* status reply; a refused request is flagged by clearing the reply bit's neighbour 0x40 */
    uint16_t m = jbus_build_status(a, (uint8_t)(ok ? instr : (instr | 0x40u)), b->tx);
    reply_at(a, m, t_reply);
}

void jbus_poll(app_t *a)
{
    jbus_t *b = &a->bus;
    while (b->tail != b->head) {
        uint16_t v = b->ring[b->tail];
        uint32_t t = b->ring_t[b->tail];
        b->tail = (uint16_t)((b->tail + 1u) % JBUS_RING);
        if (v & 0x100u) {
            if (b->st != P_SYNC1) b->gap_resync++;
            b->st = P_SYNC1;
            continue;
        }
        uint8_t c = (uint8_t)v;
        switch (b->st) {
        case P_SYNC1: if (c == 0xAA) b->st = P_SYNC2; break;
        case P_SYNC2: b->st = c == 0x55 ? P_ID : (c == 0xAA ? P_SYNC2 : P_SYNC1); break;
        case P_ID: b->buf[0] = c; b->st = P_LEN; break;
        case P_LEN:
            if (c < 1u || c > JBUS_MAX_FRAME - 6u) { b->len_err++; b->st = P_SYNC1; break; }
            b->buf[1] = c; b->n = 2; b->need = (uint16_t)(2u + c); b->st = P_DATA;
            break;
        case P_DATA:
            b->buf[b->n++] = c;
            if (b->n >= b->need) b->st = P_CRC1;
            break;
        case P_CRC1: b->buf[b->n] = c; b->st = P_CRC2; break;
        case P_CRC2: {
            uint16_t got = (uint16_t)(b->buf[b->n] | (c << 8));
            b->st = P_SYNC1;
            if (got != crc16_modbus(b->buf, b->n)) { b->crc_err++; break; }
            b->frames_ok++;
            dispatch(a, t);
            break;
        }
        default: b->st = P_SYNC1; break;
        }
    }
}
