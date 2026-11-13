/* Unit tests for the firmware modules, against a stub HAL (no motor model).
 * The closed-loop behaviour is covered by the HIL scenarios in hil/. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "joint.h"
#include "sim.h"

static int n_pass, n_fail;
#define CHECK(cond) do { if (cond) n_pass++; else { n_fail++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

/* ---------------- stub HAL ---------------- */
typedef struct {
    uint16_t adc[3], vbus, ntc, enc;
    uint8_t halls;
    float duty[3];
    bool pwm;
    uint8_t tx[256]; uint16_t tx_n; uint32_t tx_t; int tx_count;
    uint32_t now;
    int kicks;
    uint8_t flash[256]; int flash_ok;
} stub_t;

static void s_adc(void *c, uint16_t r[3], uint16_t *v) { stub_t *s = c; memcpy(r, s->adc, sizeof s->adc); *v = s->vbus; }
static uint16_t s_ntc(void *c) { return ((stub_t *)c)->ntc; }
static uint16_t s_enc(void *c) { return ((stub_t *)c)->enc; }
static uint8_t s_halls(void *c) { return ((stub_t *)c)->halls; }
static void s_duty(void *c, const float d[3]) { memcpy(((stub_t *)c)->duty, d, 3 * sizeof(float)); }
static void s_pwm(void *c, bool on) { ((stub_t *)c)->pwm = on; }
static void s_send(void *c, const uint8_t *b, uint16_t n, uint32_t t)
{
    stub_t *s = c;
    memcpy(s->tx, b, n); s->tx_n = n; s->tx_t = t; s->tx_count++;
}
static uint32_t s_micros(void *c) { return ((stub_t *)c)->now; }
static void s_kick(void *c) { ((stub_t *)c)->kicks++; }
static bool s_fw(void *c, const void *d, uint16_t n) { stub_t *s = c; memcpy(s->flash, d, n); s->flash_ok = 1; return true; }
static bool s_fr(void *c, void *d, uint16_t n) { stub_t *s = c; memcpy(d, s->flash, n); return true; }
static void s_nop(void *c) { (void)c; }
static const hal_ops_t STUB = { s_adc, s_ntc, s_enc, s_halls, s_duty, s_pwm, s_send, s_micros, s_kick, s_fw, s_fr, s_nop, s_nop };

static stub_t S;
static app_t A;

static void fresh(void)
{
    memset(&S, 0, sizeof S);
    memset(S.flash, 0xFF, sizeof S.flash);
    S.adc[0] = S.adc[1] = S.adc[2] = 2048;
    S.vbus = (uint16_t)(24.0f * VBUS_COUNTS_PER_V);
    S.ntc = 2048;
    S.halls = 5;
    hal_t h = { &STUB, &S };
    app_init(&A, h);
}

static void boot(void)
{
    for (int k = 0; k < 300; k++) app_isr(&A);
    app_ctrl_tick(&A);                       /* first control tick averages the boot samples */
}

static void fresh_enc(uint16_t raw)
{
    fresh();
    S.enc = raw;
    hal_t h = { &STUB, &S };
    app_init(&A, h);
}

/* ---------------- tests ---------------- */
static void test_math(void)
{
    double worst = 0;
    for (int k = -20000; k <= 20000; k++) {
        float x = (float)k * 0.0031f;
        float s, c;
        fsincos(x, &s, &c);
        worst = fmax(worst, fmax(fabs(s - sin((double)x)), fabs(c - cos((double)x))));
        float w = wrap_pi(x);
        CHECK(w >= -F_PI && w < F_PI);
    }
    CHECK(worst < 5e-6);
    NEAR(fabs(wrap_pi(3.0f * F_PI)), F_PI, 1e-5);
    NEAR(wrap_2pi(-0.5f), F_2PI - 0.5f, 1e-5);

    /* Clarke/Park round trip */
    ab_t ab = clarke(1.0f, -0.5f, -0.5f);
    NEAR(ab.a, 1.0f, 1e-6); NEAR(ab.b, 0.0f, 1e-6);
    float s, c;
    fsincos(0.7f, &s, &c);
    dq_t d = park((ab_t){ 0.3f, -1.2f }, s, c);
    ab_t back = inv_park(d, s, c);
    NEAR(back.a, 0.3f, 1e-5); NEAR(back.b, -1.2f, 1e-5);

    /* SVPWM: zero vector -> equal duties; reproduces line voltages; clips outside the hexagon */
    float du[3];
    CHECK(svpwm((ab_t){ 0, 0 }, 24.0f, DUTY_MAX, du) == 0);
    NEAR(du[0], du[1], 1e-7); NEAR(du[1], du[2], 1e-7);
    int clipped_any = 0;
    for (int k = 0; k < 360; k++) {
        float th = (float)k * F_PI / 180.0f, vmax = V_MAX_FACTOR * 24.0f;
        fsincos(th, &s, &c);
        ab_t v = { vmax * c, vmax * s };
        clipped_any |= svpwm(v, 24.0f, DUTY_MAX, du);
        float vab = (du[0] - du[1]) * 24.0f;
        float vab_want = 1.5f * v.a - 0.5f * F_SQRT3 * v.b;   /* va - vb */
        NEAR(vab, vab_want, 1e-3);
        for (int i = 0; i < 3; i++) CHECK(du[i] >= 0.0f && du[i] <= DUTY_MAX);
    }
    CHECK(!clipped_any);
    CHECK(svpwm((ab_t){ 30.0f, 0.0f }, 24.0f, DUTY_MAX, du) == 1);

    /* voltage limit keeps d first */
    dq_t v = { 3.0f, 20.0f };
    CHECK(limit_dq(&v, 10.0f) == 1);
    NEAR(v.d, 3.0f, 1e-6); NEAR(sqrt(v.d * v.d + v.q * v.q), 10.0, 1e-4);
}

static void test_crc_params(void)
{
    const uint8_t s9[] = "123456789";
    CHECK(crc16_modbus(s9, 9) == 0x4B37);
    fresh();
    CHECK(A.faults & F_PARAMS_RESET);
    CHECK(A.faults & F_NOT_CAL);
    A.prm.R = 0.31f;
    A.prm.cal_valid = 1;
    CHECK(params_save(&A));
    hal_t h = { &STUB, &S };
    app_init(&A, h);
    CHECK(!(A.faults & F_PARAMS_RESET));
    NEAR(A.prm.R, 0.31f, 1e-7);
    S.flash[9] ^= 0x01;                      /* one bit flipped in flash */
    app_init(&A, h);
    CHECK(A.faults & F_PARAMS_RESET);
    NEAR(A.prm.R, MOTOR_R_NOM, 1e-7);
}

static void test_ntc(void)
{
    NEAR(ntc_raw_to_c(2048), 25.0f, 0.6f);
    float last = 200.0f;
    for (int r = 100; r < 4000; r += 7) {
        float t = ntc_raw_to_c((uint16_t)r);
        CHECK(t <= last + 1e-4f);
        last = t;
    }
}

static void test_boot_and_state(void)
{
    fresh();
    CHECK(A.state == ST_BOOT);
    boot();
    CHECK(A.state == ST_IDLE);
    CHECK(!app_enable(&A));                  /* not calibrated */
    A.prm.cal_valid = 1;
    CHECK(app_enable(&A));
    CHECK(A.state == ST_RUN && S.pwm);
    /* control task stops: ISR cuts torque after 5 ms */
    for (int k = 0; k < 99; k++) app_isr(&A);
    CHECK(A.state == ST_RUN);
    for (int k = 0; k < 3; k++) app_isr(&A);
    CHECK(A.state == ST_FAULT && (A.faults & F_CTRL_STALL) && !S.pwm);
    CHECK(app_clear_faults(&A) && A.state == ST_IDLE);

    /* ADC offset out of range at boot */
    fresh();
    S.adc[1] = 2048 + 400;
    boot();
    CHECK(A.state == ST_FAULT && (A.faults & F_ADC_OFFSET));
}

static void test_overcurrent_and_voltage(void)
{
    fresh(); boot();
    A.prm.cal_valid = 1;
    CHECK(app_enable(&A));
    S.adc[0] = (uint16_t)(2048 + 16.0f * I_COUNTS_PER_A);   /* 16 A on phase A */
    S.adc[1] = (uint16_t)(2048 - 8.0f * I_COUNTS_PER_A);
    S.adc[2] = (uint16_t)(2048 - 8.0f * I_COUNTS_PER_A);
    app_isr(&A);
    CHECK(A.state == ST_FAULT && (A.faults & F_OVERCURRENT) && !S.pwm);

    fresh(); boot();
    S.vbus = (uint16_t)(31.0f * VBUS_COUNTS_PER_V);
    for (int ms = 0; ms < 2; ms++) { for (int k = 0; k < 20; k++) app_isr(&A); app_ctrl_tick(&A); }
    CHECK(A.faults & F_OVERVOLT);
    fresh(); boot();
    S.vbus = (uint16_t)(17.0f * VBUS_COUNTS_PER_V);
    for (int ms = 0; ms < 9; ms++) { for (int k = 0; k < 20; k++) app_isr(&A); app_ctrl_tick(&A); }
    CHECK(!(A.faults & F_UNDERVOLT));        /* debounced: 10 ms */
    for (int k = 0; k < 20; k++) app_isr(&A);
    app_ctrl_tick(&A);
    CHECK(A.faults & F_UNDERVOLT);
}

static void test_current_loop(void)
{
    foc_t f;
    memset(&f, 0, sizeof f);
    foc_reset(&f);
    foc_set_gains(&f, 0.25f, 0.25e-3f, 1000.0f);
    NEAR(f.kp, 0.25e-3f * F_2PI * 1000.0f, 1e-4);
    NEAR(f.ki, 0.25f * F_2PI * 1000.0f, 1e-2);
    f.vbus = 24.0f;
    float du[3];
    foc_current_loop(&f, 0.3f, 0.0f, du);
    NEAR(f.vd, 0.0f, 1e-6); NEAR(f.vq, 0.0f, 1e-6);
    /* decoupling: at speed with zero current the q voltage is the back-EMF */
    foc_reset(&f);
    foc_set_gains(&f, 0.25f, 0.25e-3f, 1000.0f);
    f.vbus = 24.0f;
    foc_current_loop(&f, 0.0f, 500.0f, du);
    NEAR(f.vq, 500.0f * MOTOR_FLUX, 1e-4);
    /* anti-windup: a request it cannot reach does not wind the integrator past the limit */
    foc_reset(&f);
    foc_set_gains(&f, 0.25f, 0.25e-3f, 1000.0f);
    f.vbus = 24.0f;
    f.iq_ref = 1000.0f;
    for (int k = 0; k < 5000; k++) foc_current_loop(&f, 0.0f, 0.0f, du);
    CHECK(f.sat);
    CHECK(f.int_q <= V_MAX_FACTOR * 24.0f + 1e-4f);
}

static void test_encoder(void)
{
    fresh_enc(65000); boot();
    A.prm.cal_valid = 1;
    /* constant speed: 20 counts per period = 400k counts/s */
    uint16_t raw = 65000;
    for (int k = 0; k < 4000; k++) { raw = (uint16_t)(raw + 20u); S.enc = raw; app_isr(&A); }
    NEAR(A.enc.pll_vel, 20.0f * PWM_HZ, 2.0f);
    CHECK(A.enc.count == 80000);             /* wrapped the 16-bit timer cleanly */
    /* sectors from codes */
    for (int k = 0; k < 6; k++) CHECK(hall_sector(&A.prm, A.prm.code_of_sector[k]) == k);
    CHECK(hall_sector(&A.prm, 0) < 0 && hall_sector(&A.prm, 7) < 0);
}

static void test_alignment(void)
{
    /* synthetic rotor turning slowly forward; halls switch at the default boundaries;
     * the encoder count is offset arbitrarily: alignment must recover the electrical angle */
    const int off = 1234;
    double th = 0.1;                         /* electrical, rad */
    fresh_enc((uint16_t)((int)floor(th / POLE_PAIRS * ENC_CPR / (2 * M_PI)) + off));
    A.prm.cal_valid = 1;
    for (int k = 0; k < 400; k++) app_isr(&A);
    for (int k = 0; k < 20000; k++) {
        th += 0.0005;
        double m = th / POLE_PAIRS;
        S.enc = (uint16_t)((int)floor(m * ENC_CPR / (2 * M_PI)) + off);
        int sec = (int)floor(fmod(th, 2 * M_PI) / (M_PI / 3));
        S.halls = A.prm.code_of_sector[sec];
        app_isr(&A);
    }
    CHECK(A.enc.aligned);
    CHECK(A.enc.src == ANG_ENCODER);
    double err = fmod((double)A.enc.theta_e - th + 9 * M_PI, 2 * M_PI) - M_PI;
    CHECK(fabs(err) < 0.02);
    CHECK(!A.enc.fault);
    /* encoder stops counting while the halls keep changing: fault within 2 edges */
    uint16_t frozen = S.enc;
    int k = 0;
    for (; k < 40000 && !A.enc.fault; k++) {
        th += 0.0005;
        int sec = (int)floor(fmod(th, 2 * M_PI) / (M_PI / 3));
        S.halls = A.prm.code_of_sector[sec];
        S.enc = frozen;
        app_isr(&A);
    }
    CHECK(A.enc.fault);
    CHECK(A.faults & F_ENCODER);
    CHECK(A.enc.src == ANG_HALL);
    CHECK(k < (int)(2.5 * (M_PI / 3) / 0.0005));
}

static void test_ctrl_modes(void)
{
    fresh(); boot();
    A.prm.cal_valid = 1;
    app_enable(&A);
    A.enc.pos = 1.0f;
    A.enc.vel = 2.0f;
    app_set_command(&A, MODE_IMPEDANCE, 1.5f, 0.0f, 0.05f, 0.5f, 0.01f);
    NEAR(ctrl_torque(&A), 0.5f * 0.5f + 0.01f * (0.0f - 2.0f) + 0.05f, 1e-6);
    app_set_command(&A, MODE_TORQUE, 0, 0, 50.0f, 0, 0);
    NEAR(ctrl_torque(&A), MOTOR_KT * A.ctrl.i_lim_eff, 1e-6);      /* clamped */
    app_set_command(&A, MODE_POSITION, 1.0f, 0.0f, 0.0f, 0, 0);
    A.enc.vel = 0.0f;
    NEAR(ctrl_torque(&A), 0.0f, 1e-6);       /* at the target, at rest */
}

static void test_derating(void)
{
    fresh(); boot();
    A.prot.t_ntc = 25.0f;
    S.ntc = 368;                             /* 100 C */
    A.prot.dT_model = 10.0f;                 /* winding estimate 110 C */
    app_monitor(&A);
    NEAR(A.prot.derate, 0.5f, 0.03f);
    CHECK(A.faults & F_DERATING);
    A.prot.ntc_only = 1;
    app_monitor(&A);
    NEAR(A.prot.derate, 1.0f, 0.03f);        /* the NTC alone says 100 C: no derating yet */
}

/* ---------------- bus ---------------- */
static void feed(const uint8_t *b, int n, uint32_t t0)
{
    for (int i = 0; i < n; i++) app_uart_rx(&A, b[i], t0 + (uint32_t)i * 3u);
    app_comm_poll(&A);
}

static int mkframe(uint8_t *o, uint8_t id, uint8_t instr, const uint8_t *d, int n)
{
    o[0] = 0xAA; o[1] = 0x55; o[2] = id; o[3] = (uint8_t)(n + 1); o[4] = instr;
    memcpy(o + 5, d, (size_t)n);
    uint16_t c = crc16_modbus(o + 2, (uint16_t)(3 + n));
    o[5 + n] = (uint8_t)c; o[6 + n] = (uint8_t)(c >> 8);
    return 7 + n;
}

static void test_bus(void)
{
    fresh(); boot();
    uint8_t f[128], d[128];
    int n = mkframe(f, 1, 0x01, d, 0);       /* PING */
    S.tx_count = 0;
    feed(f, n, 1000);
    CHECK(S.tx_count == 1 && S.tx[4] == 0x81 && S.tx[2] == 1);
    CHECK(S.tx_t == 1000u + (uint32_t)(n - 1) * 3u + JBUS_TURNAROUND_US);

    n = mkframe(f, 2, 0x01, d, 0);           /* someone else's */
    feed(f, n, 5000);
    CHECK(S.tx_count == 1);
    n = mkframe(f, 1, 0x01, d, 0);
    f[n - 1] ^= 0xFF;                        /* bad CRC */
    feed(f, n, 9000);
    CHECK(S.tx_count == 1 && A.bus.crc_err == 1);

    /* SYNC_CMD: three entries, ours is second -> reply in slot 1 */
    A.prm.cal_valid = 1;
    app_enable(&A);
    d[0] = 3;
    for (int k = 0; k < 3; k++) {
        uint8_t *e = &d[1 + 16 * k];
        memset(e, 0, 16);
        e[0] = (uint8_t)(k == 1 ? 1 : 7 + k);
        e[1] = MODE_VELOCITY;
        float v = 12.5f;
        memcpy(&e[6], &v, 4);
    }
    n = mkframe(f, 0xFE, 0x10, d, 1 + 48);
    feed(f, n, 20000);
    CHECK(A.ctrl.mode == MODE_VELOCITY);
    NEAR(A.ctrl.vel_ref, 12.5f, 1e-6);
    uint32_t slot = (24u * 10000000u) / JBUS_BAUD + 1u + JBUS_GUARD_US;
    CHECK(S.tx_t == 20000u + (uint32_t)(n - 1) * 3u + JBUS_TURNAROUND_US + slot);
    CHECK(S.tx[4] == 0x90);

    /* WRITE a register, READ it back */
    float kp = 0.02f;
    d[0] = 5; d[1] = 1; memcpy(&d[2], &kp, 4);
    n = mkframe(f, 1, 0x03, d, 6);
    feed(f, n, 30000);
    NEAR(A.prm.vel_kp, 0.02f, 1e-7);
    d[0] = 5; d[1] = 1;
    n = mkframe(f, 1, 0x02, d, 2);
    feed(f, n, 40000);
    float back;
    memcpy(&back, &S.tx[5], 4);
    NEAR(back, 0.02f, 1e-7);
    /* out-of-range write is refused (reply has bit 0x40) and ignored */
    float bad = 5.0f;
    d[0] = 5; d[1] = 1; memcpy(&d[2], &bad, 4);
    n = mkframe(f, 1, 0x03, d, 6);
    feed(f, n, 50000);
    CHECK((S.tx[4] & 0x40) && A.prm.vel_kp < 0.1f);

    /* a frame cut short then a new one after an idle gap: the parser resyncs */
    n = mkframe(f, 1, 0x01, d, 0);
    feed(f, 4, 60000);
    int before = S.tx_count;
    feed(f, n, 61000);
    CHECK(S.tx_count == before + 1 && A.bus.gap_resync >= 1);
}

static void test_fuzz(void)
{
    fresh(); boot();
    srand(1);
    uint32_t t = 0;
    int replies0 = S.tx_count;
    for (int k = 0; k < 2000000; k++) {
        t += (rand() % 50 == 0) ? 40u : 3u;
        app_uart_rx(&A, (uint8_t)rand(), t);
        if ((k & 63) == 0) app_comm_poll(&A);
    }
    app_comm_poll(&A);
    printf("  fuzz: 2,000,000 random bytes: %u frames accepted, %u CRC rejects, %u length rejects\n",
           (unsigned)A.bus.frames_ok, (unsigned)A.bus.crc_err, (unsigned)A.bus.len_err);
    CHECK(A.bus.frames_ok <= 2);             /* ~1 in 65536 header+length+CRC matches */
    CHECK(A.bus.crc_err > 0);
    CHECK(A.state != ST_RUN);                /* random data never enabled the drive */
    (void)replies0;

    /* corrupted copies of a valid frame: 1-3 flipped bits anywhere after the header must never
     * be accepted (CRC-16/MODBUS detects all 1-, 2- and odd-bit errors in frames this short) */
    fresh(); boot();
    uint8_t f[128], d[32] = { 0 };
    d[0] = 5; d[1] = 1;
    float kp = 0.02f;
    memcpy(&d[2], &kp, 4);
    int n = mkframe(f, 1, 0x03, d, 6);
    uint32_t ok0 = A.bus.frames_ok;
    for (int k = 0; k < 100000; k++) {
        uint8_t g[128];
        memcpy(g, f, (size_t)n);
        int flips = 1 + k % 3, used[3] = { -1, -1, -1 };
        for (int b = 0; b < flips; b++) {
            int bit;
            do bit = 16 + rand() % ((n - 2) * 8); while (bit == used[0] || bit == used[1]);
            used[b] = bit;                   /* distinct bits: the frame really is corrupted */
            g[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        }
        t += 100;
        feed(g, n, t);
    }
    printf("  mutation: 100,000 frames with 1-3 bit errors: %u accepted, %u CRC rejects\n",
           (unsigned)(A.bus.frames_ok - ok0), (unsigned)A.bus.crc_err);
    CHECK(A.bus.frames_ok == ok0);
    NEAR(A.prm.vel_kp, 0.010f, 1e-7);       /* the register was never written */
}

/* the simulator itself (so ASan/Valgrind also cover the plant, bus and HAL code) */
static void test_sim_smoke(void)
{
    sim_t *s = sim_new(2, 11);
    sim_run(s, 0.05);
    for (int j = 0; j < 2; j++) {
        CHECK(sim_local(s, j, "cal") == 1);
        sim_run(s, 6.0);
        CHECK(sim_get(s, j, "fw.cal_valid") == 1.0);
    }
    for (int j = 0; j < 2; j++) CHECK(sim_local(s, j, "enable") == 1);
    for (int ms = 0; ms < 600; ms++) {
        for (int j = 0; j < 2; j++) sim_local_cmd(s, j, MODE_VELOCITY, 0, 15.0 + 10.0 * j, 0, 0, 0);
        sim_run(s, 0.001);
    }
    NEAR(sim_get(s, 0, "w"), 15.0, 0.5);
    NEAR(sim_get(s, 1, "w"), 25.0, 0.5);
    /* one PING over the bus */
    uint8_t f[16], d[1], rx[256];
    int n = mkframe(f, 2, 0x01, d, 0);
    sim_master_send(s, f, n, 0.0);
    sim_run(s, 0.001);
    int m = sim_master_recv(s, rx, NULL, (int)sizeof rx);
    CHECK(m == 24 && rx[2] == 2 && rx[4] == 0x81);
    sim_free(s);
}

int main(void)
{
    test_math();
    test_crc_params();
    test_ntc();
    test_boot_and_state();
    test_overcurrent_and_voltage();
    test_current_loop();
    test_encoder();
    test_alignment();
    test_ctrl_modes();
    test_derating();
    test_bus();
    test_fuzz();
    test_sim_smoke();
    printf("%d checks passed, %d failed\n", n_pass, n_fail);
    return n_fail ? 1 : 0;
}
