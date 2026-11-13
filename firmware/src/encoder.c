/* Rotor angle and speed: incremental encoder (fine) + three hall sensors (coarse, absolute).
 *
 * An incremental encoder does not know where the rotor magnets are at power-up, so the joint
 * starts on the hall sensors and switches to the encoder at the first hall edge, where the
 * electrical angle is known from calibration. After that every hall edge cross-checks the
 * encoder; a slipping or disconnected encoder is detected and the joint keeps running on the
 * halls at reduced current instead of stopping. */
#include "joint.h"

#define PLL_BW_HZ       400.0f      /* ~9x the speed-loop bandwidth: less lag, still quiet */
#define ENC_JUMP_MAX    64          /* counts per period; more is a glitch (2500 rpm is ~9) */
#define EDGE_TOL        0.436f      /* 25 degrees electrical */
#define HALL_STALE_S    0.1f

static const float CNT2RAD = F_2PI / (float)ENC_CPR;

int8_t hall_sector(const params_t *p, uint8_t code)
{
    for (int8_t k = 0; k < 6; k++)
        if (p->code_of_sector[k] == code) return k;
    return -1;
}

void enc_init(enc_t *e, uint16_t raw, uint8_t halls)
{
    float wn = F_2PI * PLL_BW_HZ;
    e->last_raw = raw;
    e->count = 0;
    e->pll_base = 0;
    e->pll_off = 0.0f;
    e->pll_vel = 0.0f;
    e->kp_pll = 2.0f * wn;          /* critically damped */
    e->ki_pll = wn * wn;
    e->hall_code = halls;
    e->sector = -1;
    e->hall_dir = 0;
    e->hall_ok = 0;
    e->t_edge = 0;
    e->dt_edge = 0;
    e->hall_theta0 = 0.0f;
    e->hall_omega_e = 0.0f;
    e->hall_steps = 0;
    e->aligned = 0;
    e->offset_e = 0.0f;
    e->count_at_edge = 0;
    e->bad_edges = e->frozen_edges = e->fault = 0;
    e->src = ANG_HALL;
    e->src_override = ANG_NONE;
    e->theta_e = e->omega_e = e->pos = e->vel = 0.0f;
    e->pos_hall_offset = 0.0f;
    e->edges = 0;
}

static float enc_elec_angle(const enc_t *e)
{
    int32_t m = e->count % ENC_CPR;
    if (m < 0) m += ENC_CPR;
    int32_t ec = (m * POLE_PAIRS) % ENC_CPR;        /* exact integer electrical position */
    return wrap_pi((float)ec * CNT2RAD);
}

static void enc_fail(app_t *a)
{
    enc_t *e = &a->enc;
    if (e->fault) return;
    e->fault = 1;
    /* continue the position on hall steps from where the encoder was */
    e->pos_hall_offset = e->pos - (float)e->hall_steps * (F_PI / 3.0f) / (float)POLE_PAIRS;
    a->faults |= F_ENCODER;
}

static void hall_edge(app_t *a, int8_t prev, int8_t now, float th_enc, float om_enc)
{
    enc_t *e = &a->enc;
    const params_t *p = &a->prm;
    int8_t ds = (int8_t)(now - prev);
    if (ds > 3) ds = (int8_t)(ds - 6);
    if (ds < -3) ds = (int8_t)(ds + 6);
    uint32_t dt = a->isr_count - e->t_edge;
    e->t_edge = a->isr_count;
    e->edges++;
    if (ds != 1 && ds != -1) {           /* skipped a sector: noise or very fast; just resync */
        e->hall_omega_e = 0.0f;
        e->hall_dir = 0;
        e->hall_theta0 = p->hall_b[now];
        return;
    }
    int8_t dir = ds;
    e->hall_steps += dir;
    float b = dir > 0 ? p->hall_b[now] : p->hall_b[prev];   /* boundary that was crossed */
    if (e->hall_dir == dir && dt > 0 && (float)dt * TS < HALL_STALE_S)
        e->hall_omega_e = (float)dir * (F_PI / 3.0f) / ((float)dt * TS);
    else
        e->hall_omega_e = 0.0f;
    e->hall_dir = dir;
    e->dt_edge = dt;
    e->hall_theta0 = b;

    if (e->fault || !p->cal_valid) return;
    /* the edge happened somewhere in the last period: assume the middle */
    float th_edge = wrap_pi(th_enc - 0.5f * TS * om_enc);
    if (!e->aligned) {
        e->offset_e = wrap_pi(th_edge - b);
        e->aligned = 1;
        e->count_at_edge = e->count;
        return;
    }
    float r = wrap_pi(th_edge - e->offset_e - b);
    if (absf(r) > EDGE_TOL) e->bad_edges++;
    else {
        e->bad_edges = 0;
        e->offset_e = wrap_pi(e->offset_e + 0.05f * r);
    }
    if (e->count == e->count_at_edge) e->frozen_edges++;
    else e->frozen_edges = 0;
    e->count_at_edge = e->count;
    if (e->bad_edges >= 2 || e->frozen_edges >= 2) enc_fail(a);
}

void enc_update(app_t *a, uint16_t raw, uint8_t halls)
{
    enc_t *e = &a->enc;
    const params_t *p = &a->prm;

    /* ---- encoder ---- */
    int16_t d = (int16_t)(uint16_t)(raw - e->last_raw);
    e->last_raw = raw;
    if (d > ENC_JUMP_MAX || d < -ENC_JUMP_MAX) enc_fail(a);
    e->count += (int32_t)p->enc_dir * (int32_t)d;

    float err = (float)(e->count - e->pll_base) - e->pll_off;
    e->pll_off += (e->pll_vel + e->kp_pll * err) * TS;
    e->pll_vel += e->ki_pll * err * TS;
    int32_t whole = (int32_t)e->pll_off;
    e->pll_base += whole;
    e->pll_off -= (float)whole;

    float th_enc = enc_elec_angle(e);
    float om_enc = e->pll_vel * CNT2RAD * (float)POLE_PAIRS;

    /* ---- hall sensors ---- */
    int8_t s = hall_sector(p, halls);
    e->hall_code = halls;
    if (s >= 0) {
        if (e->sector >= 0 && s != e->sector) hall_edge(a, e->sector, s, th_enc, om_enc);
        else if (e->sector < 0) {
            e->hall_theta0 = p->hall_b[s];
            e->t_edge = a->isr_count;
        }
        e->sector = s;
        e->hall_ok = 1;
    } else {
        e->hall_ok = 0;                  /* 000 or 111: wiring fault or noise; keep the last */
    }

    float th_hall = 0.0f, th_six = 0.0f;
    if (e->sector >= 0) {
        float b0 = p->hall_b[e->sector];
        float width = wrap_2pi(p->hall_b[(e->sector + 1) % 6] - b0);
        th_six = wrap_pi(b0 + 0.5f * width);
        float since = (float)(a->isr_count - e->t_edge) * TS;
        float stale = 3.0f * (float)e->dt_edge * TS;
        if (stale < HALL_STALE_S) stale = HALL_STALE_S;
        if (since > stale) e->hall_omega_e = 0.0f;
        if (e->hall_omega_e == 0.0f) th_hall = th_six;
        else {
            float rel = wrap_2pi(e->hall_theta0 + e->hall_omega_e * since - b0);
            if (rel > F_PI + 0.5f * width) rel = 0.0f;   /* wrapped below the sector start */
            th_hall = wrap_pi(b0 + clampf(rel, 0.0f, width));
        }
    }

    /* ---- choose the source ---- */
    angsrc_t src;
    if (e->src_override != ANG_NONE) src = e->src_override;
    else if (e->fault || !e->aligned) src = ANG_HALL;
    else src = ANG_ENCODER;
    if (src == ANG_ENCODER && (!e->aligned || e->fault)) src = ANG_HALL;
    e->src = src;

    if (src == ANG_ENCODER) {
        e->theta_e = wrap_pi(th_enc - e->offset_e);
        e->omega_e = om_enc;
    } else if (src == ANG_SIXSTEP) {
        e->theta_e = th_six;
        e->omega_e = e->hall_omega_e;
    } else {
        e->theta_e = th_hall;
        e->omega_e = e->hall_omega_e;
    }

    if (!e->fault) {
        e->pos = (float)e->count * CNT2RAD;
        e->vel = e->pll_vel * CNT2RAD;
    } else {
        e->pos = (float)e->hall_steps * (F_PI / 3.0f) / (float)POLE_PAIRS + e->pos_hall_offset;
        e->vel = e->hall_omega_e / (float)POLE_PAIRS;
    }
}
