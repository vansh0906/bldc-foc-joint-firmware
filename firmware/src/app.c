/* The joint's state machine and the glue between the ISR, the control task and the bus.
 *
 *   BOOT  --ADC offsets ok-->  IDLE  --ENABLE (calibrated)-->  RUN  --no command-->  DAMP
 *                               |  ^                            |                      |
 *                         CALIBRATE |                         fault                  CLEAR
 *                               v  |                            v                      v
 *                               CAL ----------fault-------->  FAULT --CLEAR-->       IDLE      */
#include <string.h>

#include "joint.h"

#define BOOT_SAMPLES     256u
#define ADC_OFF_TOL      150.0f      /* counts (~1.8 A) */
#define STALL_TICKS      100u        /* 5 ms without a control tick -> torque off */

void app_trip(app_t *a, uint16_t fault)
{
    a->hal.ops->pwm_enable(a->hal.ctx, false);
    a->faults |= fault;
    a->state = ST_FAULT;
    a->foc.volt_mode = 0;
    a->foc.int_d = a->foc.int_q = 0.0f;
    a->iq_ref_shared = 0.0f;
}

void app_init(app_t *a, hal_t hal)
{
    memset(a, 0, sizeof *a);
    a->hal = hal;
    a->state = ST_BOOT;
    a->hal.ops->pwm_enable(a->hal.ctx, false);
    params_load(a);
    foc_reset(&a->foc);
    a->foc.advance = 1.5f;
    params_apply_gains(a);
    ctrl_reset(&a->ctrl);
    prot_reset(&a->prot);
    jbus_reset(&a->bus);
    enc_init(&a->enc, a->hal.ops->read_encoder(a->hal.ctx), a->hal.ops->read_halls(a->hal.ctx));
}

static void boot_isr(app_t *a, const uint16_t raw[3], uint16_t vraw)
{
    for (int k = 0; k < 3; k++) a->boot_acc[k] += (float)raw[k];
    a->foc.vbus = (float)vraw * (1.0f / VBUS_COUNTS_PER_V);
    if (++a->boot_n < BOOT_SAMPLES) return;
    for (int k = 0; k < 3; k++) {
        a->foc.i_off[k] = a->boot_acc[k] / (float)BOOT_SAMPLES;
        if (absf(a->foc.i_off[k] - ADC_MID) > ADC_OFF_TOL) { app_trip(a, F_ADC_OFFSET); return; }
    }
    a->state = ST_IDLE;
}

void app_isr(app_t *a)
{
    const hal_ops_t *h = a->hal.ops;
    void *cx = a->hal.ctx;
    uint16_t raw[3], vraw;
    float duty[3];

    a->isr_count++;
    h->read_adc(cx, raw, &vraw);
    enc_update(a, h->read_encoder(cx), h->read_halls(cx));

    if (a->state == ST_BOOT) {           /* PWM is off: no current flows, sample the offsets */
        boot_isr(a, raw, vraw);
        a->vbus_acc += a->foc.vbus;
        a->vbus_n++;
        return;
    }
    foc_measure(&a->foc, raw, vraw);
    a->vbus_acc += a->foc.vbus;
    a->vbus_n++;

    jstate_t st = a->state;
    bool driving = st == ST_RUN || st == ST_DAMP || st == ST_CAL;
    if (!driving) return;

    if (absf(a->foc.ia) > I_OC_TRIP || absf(a->foc.ib) > I_OC_TRIP || absf(a->foc.ic) > I_OC_TRIP) {
        app_trip(a, F_OVERCURRENT);
        return;
    }
    if (st == ST_CAL) {
        foc_dq(&a->foc, a->cal.theta_cmd);       /* calibration reads this period's id/iq */
        bool done = cal_isr(a);
        if (a->state != ST_CAL) return;            /* calibration failed and tripped */
        if (done) {
            h->pwm_enable(cx, false);
            foc_reset(&a->foc);
            a->state = ST_IDLE;
            return;
        }
        foc_current_loop(&a->foc, a->cal.theta_cmd, 0.0f, duty);
        h->set_duty(cx, duty);
        return;
    }
    /* RUN / DAMP: if the control task stops ticking, the last torque request would stay applied
     * until the watchdog resets the chip; cut the torque here instead */
    if (a->isr_count - a->ctrl_stamp > STALL_TICKS) {
        app_trip(a, F_CTRL_STALL);
        return;
    }
    a->foc.id_ref = a->id_ref_shared;
    a->foc.iq_ref = a->iq_ref_shared;
    /* regenerative braking pumps energy into the bus capacitor; with a supply that cannot
     * absorb it the bus rises ~10 V per ms, too fast for the 1 kHz loop: limit braking current
     * here, every period, from the instantaneous bus voltage */
    if (!a->prot.regen_off && a->foc.iq_ref * a->enc.omega_e < 0.0f) {
        float k = clampf((VBUS_REGEN_ZERO - a->foc.vbus) / (VBUS_REGEN_ZERO - VBUS_REGEN_START), 0.0f, 1.0f);
        a->foc.iq_ref *= k;
        if (k < 1.0f) a->regen_hits++;
    }
    foc_current_loop(&a->foc, a->enc.theta_e, a->enc.omega_e, duty);
    h->set_duty(cx, duty);
}

void app_ctrl_tick(app_t *a)
{
    ctrl_t *c = &a->ctrl;
    c->ms++;
    a->ctrl_stamp = a->isr_count;
    if (a->vbus_n) {
        a->prot.vbus_f = a->vbus_acc / (float)a->vbus_n;
        a->vbus_acc = 0.0f;
        a->vbus_n = 0;
    }
    if (a->state != ST_BOOT) prot_fast(a);

    if (a->state == ST_RUN && (uint32_t)(c->ms - c->last_cmd_ms) > a->prm.comm_timeout_ms) {
        a->state = ST_DAMP;
        a->faults |= F_COMM_TIMEOUT;
    }

    float tau = 0.0f;
    if (a->state == ST_RUN) tau = ctrl_torque(a);
    else if (a->state == ST_DAMP) {
        float lim = 0.5f * MOTOR_KT * c->i_lim_eff;
        tau = clampf(-a->prm.kd_safe * a->enc.vel, -lim, lim);
        c->tau_cmd = tau;
    }
    float iq = 0.0f;
    if (a->state == ST_RUN || a->state == ST_DAMP) iq = prot_limit_iq(a, tau / MOTOR_KT);
    c->iq_cmd = iq;
    a->iq_ref_shared = iq;
    a->id_ref_shared = 0.0f;
    prot_account_loss(a, a->foc.id, a->foc.iq);
    a->alive |= 1u;
}

void app_monitor(app_t *a)
{
    if (a->state != ST_BOOT) prot_slow(a);
    if ((a->alive & 3u) == 3u) {          /* control and comm tasks both ran since last kick */
        a->hal.ops->wdg_kick(a->hal.ctx);
        a->wdg_kicks++;
        a->alive = 0;
    }
}

void app_comm_poll(app_t *a)
{
    jbus_poll(a);
    a->alive |= 2u;
}

bool app_enable(app_t *a)
{
    if (a->state != ST_IDLE || (a->faults & F_LATCHED_MASK) || !a->prm.cal_valid) return false;
    a->hal.ops->irq_disable(a->hal.ctx);
    foc_reset(&a->foc);
    a->foc.advance = 1.5f;
    params_apply_gains(a);
    ctrl_t *c = &a->ctrl;
    c->mode = MODE_TORQUE;
    c->pos_ref = a->enc.pos;
    c->vel_ref = c->tau_ff = c->kp = c->kd = 0.0f;
    c->vel_int = 0.0f;
    c->last_cmd_ms = c->ms;
    a->iq_ref_shared = 0.0f;
    a->ctrl_stamp = a->isr_count;
    a->faults &= (uint16_t)~F_COMM_TIMEOUT;
    a->state = ST_RUN;
    /* flying start: the first duties apply the back-EMF voltage itself, so enabling on a moving
     * rotor drives no current (the current loop's feed-forward takes over from the next period) */
    {
        dq_t v = { 0.0f, a->no_preload ? 0.0f : a->enc.omega_e * a->foc.flux };
        float sn, cs;
        fsincos(a->enc.theta_e + TS * a->enc.omega_e, &sn, &cs);
        svpwm(inv_park(v, sn, cs), a->foc.vbus, DUTY_MAX, a->foc.duty);
    }
    a->hal.ops->set_duty(a->hal.ctx, a->foc.duty);
    a->hal.ops->pwm_enable(a->hal.ctx, true);
    a->hal.ops->irq_enable(a->hal.ctx);
    return true;
}

void app_disable(app_t *a)
{
    a->hal.ops->pwm_enable(a->hal.ctx, false);
    if (a->state == ST_RUN || a->state == ST_DAMP || a->state == ST_CAL) a->state = ST_IDLE;
    foc_reset(&a->foc);
    a->iq_ref_shared = 0.0f;
}

bool app_clear_faults(app_t *a)
{
    if (a->state == ST_RUN || a->state == ST_CAL || a->state == ST_BOOT) return false;
    a->hal.ops->pwm_enable(a->hal.ctx, false);
    a->faults &= (uint16_t)~(F_LATCHED_MASK | F_COMM_TIMEOUT);
    a->prot.ov_cnt = a->prot.uv_cnt = 0;
    foc_reset(&a->foc);
    a->state = ST_IDLE;
    return true;
}

bool app_start_cal(app_t *a)
{
    if (a->state != ST_IDLE || (a->faults & F_LATCHED_MASK)) return false;
    cal_start(a);
    a->ctrl_stamp = a->isr_count;
    a->state = ST_CAL;
    a->hal.ops->set_duty(a->hal.ctx, a->foc.duty);
    a->hal.ops->pwm_enable(a->hal.ctx, true);
    return true;
}

void app_set_command(app_t *a, jmode_t mode, float pos, float vel, float tau_ff, float kp, float kd)
{
    ctrl_t *c = &a->ctrl;
    a->hal.ops->irq_disable(a->hal.ctx);
    if (mode != c->mode) c->vel_int = 0.0f;
    c->mode = mode;
    c->pos_ref = pos;
    c->vel_ref = vel;
    c->tau_ff = tau_ff;
    c->kp = kp;
    c->kd = kd;
    c->last_cmd_ms = c->ms;
    a->hal.ops->irq_enable(a->hal.ctx);
}
