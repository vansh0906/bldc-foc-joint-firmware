/* Hardware-in-the-loop simulator: N joints (firmware + plant each) on one RS-485 bus with a
 * host port. The firmware is the real firmware source; only hal.h is implemented here.
 *
 * Timing per joint follows the target: ADC sample at the PWM center -> ISR -> new duties take
 * effect next period; 1 kHz control task; 100 Hz monitor; comm task runs when a byte arrives.
 * The bus is simulated byte by byte at the configured baud rate, with collisions and bit errors.
 * Exported with C linkage for Python (ctypes) and for the C tests and tools. */
#include "sim.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_EV 8192

/* ---------------- HAL on the plant ---------------- */

static void h_read_adc(void *c, uint16_t r[3], uint16_t *v) { plant_adc(&((sjoint_t *)c)->plant, r, v); }
static uint16_t h_read_ntc(void *c) { return plant_ntc_raw(&((sjoint_t *)c)->plant); }
static uint16_t h_read_enc(void *c) { return plant_encoder(&((sjoint_t *)c)->plant); }
static uint8_t h_read_halls(void *c) { return plant_halls(&((sjoint_t *)c)->plant); }
static void h_set_duty(void *c, const float d[3])
{
    plant_t *p = &((sjoint_t *)c)->plant;
    for (int k = 0; k < 3; k++) p->duty_next[k] = d[k];
    p->have_next = 1;
}
static void h_pwm(void *c, bool on)
{
    sjoint_t *j = c;
    j->plant.pwm_on = on ? 1 : 0;
    if (on) {
        /* the target writes the compare registers and forces an update before enabling the
         * outputs, so the first period already uses the duties set before pwm_enable */
        if (j->plant.have_next) {
            memcpy(j->plant.duty, j->plant.duty_next, sizeof j->plant.duty);
            j->plant.have_next = 0;
        }
        j->pwm_on_events++;
    }
}
static uint32_t h_micros(void *c) { return (uint32_t)(((sjoint_t *)c)->sim->now_us); }
static void h_uart_send(void *c, const uint8_t *buf, uint16_t n, uint32_t t_start)
{
    sjoint_t *j = c;
    bus_tx(j->sim, j->node, buf, n, (double)t_start);
}
static void h_wdg(void *c) { sjoint_t *j = c; j->last_kick = j->sim->t; }
static bool h_flash_write(void *c, const void *d, uint16_t n)
{
    sjoint_t *j = c;
    if (n > sizeof j->flash) return false;
    memcpy(j->flash, d, n);
    j->flash_writes++;
    return true;
}
static bool h_flash_read(void *c, void *d, uint16_t n)
{
    sjoint_t *j = c;
    if (n > sizeof j->flash) return false;
    memcpy(d, j->flash, n);
    return true;
}
static void h_irq_off(void *c) { (void)c; }
static void h_irq_on(void *c) { (void)c; }

static const hal_ops_t SIM_HAL = {
    h_read_adc, h_read_ntc, h_read_enc, h_read_halls, h_set_duty, h_pwm, h_uart_send, h_micros,
    h_wdg, h_flash_write, h_flash_read, h_irq_off, h_irq_on
};

/* ---------------- bus ---------------- */

static double byte_us(const sim_t *s) { return 10.0e6 / s->baud; }

static void ev_push(sim_t *s, double t, int src, uint8_t b, int bad)
{
    if (s->nev >= MAX_EV) { s->ev_dropped++; return; }
    int i = s->nev++;
    while (i > 0 && s->ev[i - 1].t > t) { s->ev[i] = s->ev[i - 1]; i--; }   /* keep sorted */
    s->ev[i].t = t; s->ev[i].src = src; s->ev[i].b = b; s->ev[i].bad = bad;
}

void bus_tx(sim_t *s, int node, const uint8_t *buf, int n, double t_start_us)
{
    if (t_start_us < s->now_us) { s->late_tx++; t_start_us = s->now_us; }
    double bt = byte_us(s);
    double t_end = t_start_us + n * bt;
    /* half duplex: two drivers on at once garble both frames */
    int collided = 0;
    for (int k = 0; k < s->ntx; k++) {
        if (s->tx[k].t1 > t_start_us && s->tx[k].t0 < t_end && s->tx[k].node != node) collided = 1;
    }
    if (collided) {
        s->collisions++;
        for (int i = 0; i < s->nev; i++)
            if (s->ev[i].t > t_start_us && s->ev[i].t <= t_end + bt) s->ev[i].bad = 1;
    }
    /* remember the busy interval (keep a short history) */
    if (s->ntx == 16) { memmove(s->tx, s->tx + 1, 15 * sizeof s->tx[0]); s->ntx = 15; }
    s->tx[s->ntx].t0 = t_start_us; s->tx[s->ntx].t1 = t_end; s->tx[s->ntx].node = node; s->ntx++;
    for (int i = 0; i < n; i++) {
        uint8_t b = buf[i];
        int bad = collided;
        if (s->ber > 0.0)
            for (int bit = 0; bit < 8; bit++)
                if (rng_uniform(&s->rng) < s->ber) { b ^= (uint8_t)(1u << bit); s->bit_errors++; }
        ev_push(s, t_start_us + (i + 1) * bt, node, b, bad);
    }
    s->tx_bytes += (uint64_t)n;
    if (t_end > s->bus_busy_until) s->bus_busy_until = t_end;
    s->bus_busy_us += n * bt;
}

static void deliver(sim_t *s, double t_until_us)
{
    int k = 0;
    while (k < s->nev && s->ev[k].t <= t_until_us) {
        ev_t e = s->ev[k++];
        uint8_t b = e.bad ? (uint8_t)(e.b ^ 0xA5u) : e.b;
        double save = s->now_us;
        s->now_us = e.t;
        if (e.src != 0 && s->mrx_n < (int)(sizeof s->mrx / sizeof s->mrx[0])) {   /* to the host port */
            s->mrx[s->mrx_n] = b;
            s->mrx_t[s->mrx_n] = e.t;
            s->mrx_n++;
        }
        for (int j = 0; j < s->nj; j++) {
            sjoint_t *J = &s->j[j];
            if (J->node == e.src || J->dead) continue;        /* no echo of our own bytes */
            app_uart_rx(&J->app, b, (uint32_t)e.t);
            app_comm_poll(&J->app);                            /* comm task wakes on RX */
        }
        s->now_us = save;
    }
    if (k) { memmove(s->ev, s->ev + k, (size_t)(s->nev - k) * sizeof s->ev[0]); s->nev -= k; }
}

/* ---------------- joints ---------------- */

static void joint_boot(sjoint_t *J)
{
    hal_t h = { &SIM_HAL, J };
    app_init(&J->app, h);
    J->tick = 0;
    J->last_kick = J->sim->t;
}

sim_t *sim_new(int nj, uint64_t seed)
{
    if (nj < 1 || nj > SIM_MAX_J) return NULL;
    sim_t *s = calloc(1, sizeof *s);
    s->nj = nj;
    s->baud = 4000000.0;
    s->rng.s = seed * 7919 + 17;
    s->wdg_timeout = 0.05;
    for (int j = 0; j < nj; j++) {
        sjoint_t *J = &s->j[j];
        J->sim = s;
        J->node = j + 1;
        plant_init(&J->plant, seed * 1000 + (uint64_t)j, 1);
        memset(J->flash, 0xFF, sizeof J->flash);
        joint_boot(J);
        J->app.prm.node_id = (uint8_t)(j + 1);
        params_save(&J->app);            /* factory programming: node id in flash */
        joint_boot(J);
        J->log_div = 20;
        J->log_cap = 0;
    }
    return s;
}

void sim_free(sim_t *s)
{
    if (!s) return;
    for (int j = 0; j < s->nj; j++) free(s->j[j].log);
    free(s);
}

static void log_row(sjoint_t *J)
{
    if (!J->log || J->log_n >= J->log_cap) return;
    plant_t *p = &J->plant;
    app_t *a = &J->app;
    double *r = &J->log[(size_t)J->log_n * LOG_COLS];
    double th_true = fmod(p->pp * J->th_isr, 2.0 * M_PI);   /* rotor angle when the ISR sampled */
    double err = a->enc.theta_e - th_true;
    while (err > M_PI) err -= 2 * M_PI;
    while (err < -M_PI) err += 2 * M_PI;
    r[0] = p->t; r[1] = p->th; r[2] = p->w; r[3] = a->enc.pos; r[4] = a->enc.vel;
    r[5] = a->foc.iq_ref; r[6] = a->foc.iq; r[7] = p->iq; r[8] = p->id; r[9] = p->Te;
    r[10] = p->vbus; r[11] = p->Tw; r[12] = p->Tntc; r[13] = a->prot.t_wind_est;
    r[14] = a->state; r[15] = a->faults; r[16] = err; r[17] = a->enc.src;
    r[18] = a->foc.id; r[19] = p->ia;
    J->log_n++;
}

static void joint_period(sim_t *s, sjoint_t *J)
{
    if (J->dead) return;
    app_t *a = &J->app;
    J->th_isr = J->plant.th;
    app_isr(a);                                         /* samples now, duties for next period */
    if (++J->tick % CTRL_DIV == 0) {
        if (!J->hang_ctrl) app_ctrl_tick(a);
        app_comm_poll(a);                               /* comm task also wakes on its 1 ms timeout */
        if ((J->tick / CTRL_DIV) % MON_DIV == 0) app_monitor(a);
        /* independent watchdog: a reset turns the PWM off and reboots the firmware */
        if (s->t - J->last_kick > s->wdg_timeout) {
            J->wdg_resets++;
            J->plant.pwm_on = 0;
            J->hang_ctrl = 0;
            joint_boot(J);
        }
    }
    plant_step(&J->plant, TS);
    if (J->log && (J->tick % (uint32_t)J->log_div == 0)) log_row(J);
}

void sim_run(sim_t *s, double seconds)
{
    long n = lround(seconds * PWM_HZ);
    for (long k = 0; k < n; k++) {
        deliver(s, s->now_us);
        for (int j = 0; j < s->nj; j++) joint_period(s, &s->j[j]);
        s->steps++;
        s->t = (double)s->steps / PWM_HZ;
        s->now_us = s->t * 1e6;
    }
    deliver(s, s->now_us);
}

/* ---------------- host port ---------------- */

void sim_master_send(sim_t *s, const uint8_t *buf, int n, double delay_us)
{
    bus_tx(s, 0, buf, n, s->now_us + delay_us);
}

int sim_master_recv(sim_t *s, uint8_t *buf, double *t, int max)
{
    int n = s->mrx_n < max ? s->mrx_n : max;
    memcpy(buf, s->mrx, (size_t)n);
    if (t) memcpy(t, s->mrx_t, (size_t)n * sizeof(double));
    memmove(s->mrx, s->mrx + n, (size_t)(s->mrx_n - n));
    memmove(s->mrx_t, s->mrx_t + n, (size_t)(s->mrx_n - n) * sizeof(double));
    s->mrx_n -= n;
    return n;
}

/* ---------------- test hooks ---------------- */

void sim_local_cmd(sim_t *s, int j, int mode, double pos, double vel, double tau, double kp, double kd)
{
    app_set_command(&s->j[j].app, (jmode_t)mode, (float)pos, (float)vel, (float)tau, (float)kp, (float)kd);
}

int sim_local(sim_t *s, int j, const char *what)
{
    sjoint_t *J = &s->j[j];
    app_t *a = &J->app;
    if (!strcmp(what, "enable")) return app_enable(a);
    if (!strcmp(what, "disable")) { app_disable(a); return 1; }
    if (!strcmp(what, "clear")) return app_clear_faults(a);
    if (!strcmp(what, "cal")) return app_start_cal(a);
    if (!strcmp(what, "save")) return params_save(a);
    if (!strcmp(what, "reboot")) { J->plant.pwm_on = 0; joint_boot(J); return 1; }
    return -1;
}

void sim_log_config(sim_t *s, int j, double rate_hz, int max_rows)
{
    sjoint_t *J = &s->j[j];
    free(J->log);
    J->log = calloc((size_t)max_rows * LOG_COLS, sizeof(double));
    J->log_cap = max_rows;
    J->log_n = 0;
    int div = (int)lround(PWM_HZ / rate_hz);
    J->log_div = div < 1 ? 1 : div;
}

int sim_log_get(sim_t *s, int j, double *out, int max_rows)
{
    sjoint_t *J = &s->j[j];
    int n = J->log_n < max_rows ? J->log_n : max_rows;
    memcpy(out, J->log, (size_t)n * LOG_COLS * sizeof(double));
    return n;
}

double sim_time(sim_t *s) { return s->t; }

#define SETP(name, field) if (!strcmp(key, name)) { p->field = v; return 0; }
#define SETI(name, field) if (!strcmp(key, name)) { p->field = (int)v; return 0; }

int sim_set(sim_t *s, int j, const char *key, double v)
{
    if (!strcmp(key, "baud")) { s->baud = v; for (int k = 0; k < s->nj; k++) s->j[k].app.prm.baud = (uint32_t)v; return 0; }
    if (!strcmp(key, "ber")) { s->ber = v; return 0; }
    if (!strcmp(key, "wdg_timeout")) { s->wdg_timeout = v; return 0; }
    sjoint_t *J = &s->j[j];
    plant_t *p = &J->plant;
    app_t *a = &J->app;
    SETP("T_load", T_load) SETP("grav", grav) SETP("spring_k", spring_k) SETP("spring_x0", spring_x0)
    SETI("locked", locked) SETI("supply_sink", supply_sink) SETI("enc_rev", enc_rev) SETP("J", J) SETP("Vs", Vs) SETI("supply_on", supply_on) SETP("Tamb", Tamb)
    SETI("enc_frozen", enc_frozen) SETI("enc_slip", enc_slip) SETI("short_phase", short_phase)
    SETI("halls_dead", halls_dead) SETP("Tc", Tc) SETP("cog_amp", cog_amp) SETP("h6", h6)
    SETP("adc_noise", adc_noise) SETP("Rbleed", Rbleed) SETP("Cbus", Cbus)
    if (!strcmp(key, "Tw")) { p->Tw = p->Ts = p->Tntc = v; return 0; }
    if (!strcmp(key, "hang_ctrl")) { J->hang_ctrl = (int)v; return 0; }
    if (!strcmp(key, "dead")) { J->dead = (int)v; return 0; }
    if (!strcmp(key, "node_id")) { a->prm.node_id = (uint8_t)v; return 0; }
    if (!strcmp(key, "flash_corrupt")) { J->flash[(int)v % (int)sizeof J->flash] ^= 0x10u; return 0; }
    if (!strcmp(key, "fw.no_preload")) { a->no_preload = (uint8_t)v; return 0; }
    if (!strcmp(key, "fw.ntc_only")) { a->prot.ntc_only = (uint8_t)v; return 0; }
    if (!strcmp(key, "fw.src")) { a->enc.src_override = (angsrc_t)(int)v; return 0; }
    if (!strcmp(key, "fw.advance")) { a->foc.advance = (float)v; return 0; }
    if (!strcmp(key, "fw.cur_bw")) { a->prm.cur_bw_hz = (float)v; params_apply_gains(a); return 0; }
    if (!strcmp(key, "fw.pll_bw")) { float wn = 6.2831853f * (float)v; a->enc.kp_pll = 2.0f * wn; a->enc.ki_pll = wn * wn; return 0; }
    if (!strcmp(key, "fw.vel_kp")) { a->prm.vel_kp = (float)v; return 0; }
    if (!strcmp(key, "fw.vel_ki")) { a->prm.vel_ki = (float)v; return 0; }
    if (!strcmp(key, "fw.pos_kp")) { a->prm.pos_kp = (float)v; return 0; }
    if (!strcmp(key, "fw.comm_timeout_ms")) { a->prm.comm_timeout_ms = (uint16_t)v; return 0; }
    if (!strcmp(key, "fw.regen_off")) { a->prot.regen_off = (uint8_t)v; return 0; }
    return -1;
}

#define GETP(name, field) if (!strcmp(key, name)) return (double)(p->field);
#define GETA(name, expr) if (!strcmp(key, name)) return (double)(expr);

double sim_get(sim_t *s, int j, const char *key)
{
    if (!strcmp(key, "t")) return s->t;
    if (!strcmp(key, "collisions")) return s->collisions;
    if (!strcmp(key, "late_tx")) return s->late_tx;
    if (!strcmp(key, "bit_errors")) return s->bit_errors;
    if (!strcmp(key, "bus_busy_us")) return s->bus_busy_us;
    sjoint_t *J = &s->j[j];
    plant_t *p = &J->plant;
    app_t *a = &J->app;
    GETP("R", R25) GETP("Ld", Ld) GETP("Lq", Lq) GETP("flux", flux) GETP("th", th) GETP("w", w)
    GETP("id", id) GETP("iq", iq) GETP("Te", Te) GETP("vbus", vbus) GETP("Tw", Tw) GETP("Ts", Ts)
    GETP("Tntc", Tntc) GETP("pwm_on", pwm_on) GETP("ia", ia)
    if (!strcmp(key, "hall_shift0")) return p->hall_shift[0];
    if (!strcmp(key, "hall_shift1")) return p->hall_shift[1];
    if (!strcmp(key, "hall_shift2")) return p->hall_shift[2];
    GETA("fw.state", a->state) GETA("fw.faults", a->faults) GETA("fw.R", a->prm.R) GETA("fw.L", a->prm.L)
    GETA("fw.pos", a->enc.pos) GETA("fw.vel", a->enc.vel) GETA("fw.iq", a->foc.iq) GETA("fw.id", a->foc.id)
    GETA("fw.iq_ref", a->foc.iq_ref) GETA("fw.src", a->enc.src) GETA("fw.aligned", a->enc.aligned)
    GETA("fw.enc_fault", a->enc.fault) GETA("fw.cal_valid", a->prm.cal_valid) GETA("fw.enc_dir", a->prm.enc_dir)
    GETA("fw.t_wind_est", a->prot.t_wind_est) GETA("fw.t_ntc", a->prot.t_ntc) GETA("fw.derate", a->prot.derate)
    GETA("fw.vbus", a->prot.vbus_f) GETA("fw.theta_e", a->enc.theta_e) GETA("fw.kp", a->foc.kp)
    GETA("fw.ki", a->foc.ki) GETA("fw.frames_ok", a->bus.frames_ok) GETA("fw.crc_err", a->bus.crc_err)
    GETA("fw.len_err", a->bus.len_err) GETA("fw.replies", a->bus.replies) GETA("fw.gap_resync", a->bus.gap_resync)
    GETA("fw.node_id", a->prm.node_id) GETA("fw.mode", a->ctrl.mode) GETA("fw.wdg_kicks", a->wdg_kicks)
    GETA("fw.offset_e", a->enc.offset_e) GETA("fw.edges", a->enc.edges) GETA("fw.i_lim", a->ctrl.i_lim_eff)
    GETA("cal.tau_acc", a->cal.tau_acc) GETA("cal.seen_fwd", a->cal.seen_fwd) GETA("cal.seen_rev", a->cal.seen_rev) GETA("cal.ncodes", a->cal.ncodes) GETA("cal.tau_n", a->cal.tau_n) GETA("cal.step", a->cal.step) GETA("cal.v1", a->cal.v1) GETA("cal.acc_i", a->cal.acc_i) GETA("cal.t", a->cal.t) GETA("cal.i1", a->cal.i1)
    GETA("wdg_resets", J->wdg_resets) GETA("flash_writes", J->flash_writes)
    if (!strncmp(key, "fw.hall_b", 9)) { int k = key[9] - '0'; if (k >= 0 && k < 6) return a->prm.hall_b[k]; }
    if (!strncmp(key, "fw.code", 7)) { int k = key[7] - '0'; if (k >= 0 && k < 6) return a->prm.code_of_sector[k]; }
    return NAN;
}
