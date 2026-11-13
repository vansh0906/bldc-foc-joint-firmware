/* Self-commissioning, run once per joint with no load on the output (see docs/BRINGUP.md).
 *
 *  1. Lock the rotor with d-axis current at a fixed angle.
 *  2. R from two current levels: R = (V2 - V1) / (I2 - I1). The two-point difference cancels
 *     the inverter dead-time voltage error, which a single V/I ratio would read as resistance.
 *  3. L from the current response to a d-axis voltage step (locked rotor, 8 repeats).
 *  4. Rotate a d-axis current vector slowly forward and back through 2.25 electrical turns:
 *     the rotor follows it, so the commanded angle is the rotor's electrical angle. Each hall
 *     transition is recorded both ways and averaged (cancels lag and sensor hysteresis).
 *     The encoder count over the sweep checks direction and pole-pair count.
 *  5. Current-loop gains from the measured R and L. */
#include "joint.h"

enum { C_LOCK, C_R1, C_R2, C_L_HOLD, C_L_STEP, C_FWD, C_REV, C_DONE };

#define T(ms) ((uint32_t)((ms) * 0.001f * PWM_HZ))
#define L_REPS      8u
#define L_DV        3.0f
#define L_SAMPLES   4u
#define SWEEP_REVS  2.25f    /* a quarter turn extra: an edge right at angle 0 is seen even with lag */

void cal_start(app_t *a)
{
    cal_t *c = &a->cal;
    c->step = C_LOCK;
    c->t = 0;
    c->acc_v = c->acc_i = 0.0f;
    c->acc_n = 0;
    c->tau_acc = 0.0f;
    c->tau_n = 0;
    c->rep = 0;
    c->theta_cmd = 0.0f;
    c->seen_fwd = c->seen_rev = 0;
    c->ncodes = 0;
    c->last_code = a->enc.hall_code;
    for (int k = 0; k < 6; k++) c->edge_fwd[k] = c->edge_rev[k] = 0.0f;
    a->foc.volt_mode = 0;
    a->foc.id_ref = 0.0f;
    a->foc.iq_ref = 0.0f;
    a->foc.int_d = a->foc.int_q = 0.0f;
    /* nominal gains are good enough to hold current while we measure */
    foc_set_gains(&a->foc, MOTOR_R_NOM, MOTOR_L_NOM, 500.0f);
}

static void fail(app_t *a)
{
    a->foc.volt_mode = 0;
    app_trip(a, F_CAL_FAILED);
}

static int code_slot(cal_t *c, uint8_t code)
{
    for (int k = 0; k < c->ncodes; k++)
        if (c->codes[k] == code) return k;
    if (c->ncodes >= 6) return -1;
    c->codes[c->ncodes] = code;
    return c->ncodes++;
}

static void finish_halls(app_t *a)
{
    cal_t *c = &a->cal;
    params_t *p = &a->prm;
    if (c->ncodes != 6 || c->seen_fwd != 0x3F || c->seen_rev != 0x3F) { fail(a); return; }
    /* order the codes by forward entry angle in [0, 2pi) */
    int order[6];
    float key[6];
    for (int k = 0; k < 6; k++) {
        order[k] = k;
        key[k] = wrap_2pi(c->edge_fwd[k]);
    }
    for (int i = 1; i < 6; i++)
        for (int j = i; j > 0 && key[order[j]] < key[order[j - 1]]; j--) {
            int t = order[j]; order[j] = order[j - 1]; order[j - 1] = t;
        }
    float b[6];
    for (int k = 0; k < 6; k++) {
        int slot = order[k];
        int prev = order[(k + 5) % 6];
        /* forward entry into sector k and reverse entry into sector k-1 cross the same edge */
        float f = c->edge_fwd[slot];
        float r = c->edge_rev[prev];
        b[k] = wrap_pi(f + 0.5f * wrap_pi(r - f));
    }
    for (int k = 0; k < 6; k++) {
        float w = wrap_2pi(b[(k + 1) % 6] - b[k]);
        if (w < 0.70f || w > 1.40f) { fail(a); return; }    /* 40..80 degrees */
        uint8_t code = c->codes[order[k]];
        if (code == 0 || code == 7) { fail(a); return; }
        p->code_of_sector[k] = code;
        p->hall_b[k] = b[k];
    }
}

static void lock_at(app_t *a, float id)
{
    a->foc.volt_mode = 0;
    a->foc.id_ref = id;
    a->foc.iq_ref = 0.0f;
}

bool cal_isr(app_t *a)
{
    cal_t *c = &a->cal;
    foc_t *f = &a->foc;
    params_t *p = &a->prm;
    c->t++;
    switch (c->step) {
    case C_LOCK:            /* ramp up and let the rotor settle at angle 0 */
        lock_at(a, CAL_I_LOW * clampf((float)c->t / (float)T(100), 0.0f, 1.0f));
        if (c->t >= T(400)) { c->step = C_R1; c->t = 0; c->acc_v = c->acc_i = 0.0f; c->acc_n = 0; }
        break;
    case C_R1:
    case C_R2: {
        float iset = c->step == C_R1 ? CAL_I_LOW : CAL_I_ALIGN;
        lock_at(a, iset);
        if (c->t > T(150)) { c->acc_v += f->vd; c->acc_i += f->id; c->acc_n++; }
        if (c->t >= T(300)) {
            float v = c->acc_v / (float)c->acc_n, i = c->acc_i / (float)c->acc_n;
            if (c->step == C_R1) { c->v1 = v; c->i1 = i; c->step = C_R2; }
            else {
                c->v2 = v; c->i2 = i;
                float R = (c->v2 - c->v1) / (c->i2 - c->i1);
                if (!(R > 0.03f && R < 3.0f)) { fail(a); return true; }
                p->R = R;
                c->step = C_L_HOLD;
            }
            c->t = 0; c->acc_v = c->acc_i = 0.0f; c->acc_n = 0;
        }
        break;
    }
    case C_L_HOLD:          /* closed loop at I_LOW, then hand over to the same voltage open loop */
        if (c->t < T(30)) lock_at(a, CAL_I_LOW);
        else if (c->t == T(30)) {
            f->volt_mode = 1;
            f->vd_ol = c->v1;
            f->vq_ol = 0.0f;
        } else if (c->t >= T(32)) {
            c->acc_i = f->id;                  /* base current just before the step */
            f->vd_ol = c->v1 + L_DV;           /* applied from the next period on */
            c->step = C_L_STEP;
            c->t = 0;
        }
        break;
    case C_L_STEP:          /* sample j has seen the step for (j-1) periods */
        if (c->t >= 2 && c->t <= 1 + L_SAMPLES) {
            float x = (f->id - c->acc_i) * p->R / L_DV;
            if (x > 0.0f && x < 0.95f) {
                float tj = (float)(c->t - 1) * TS;
                /* tau = -t / ln(1 - x); ln via the series of -ln(1-x) */
                float s = x, term = x;
                for (int n = 2; n < 40; n++) { term *= x; s += term / (float)n; }
                c->tau_acc += tj / s;
                c->tau_n++;
            }
        }
        if (c->t > 1 + L_SAMPLES) {
            f->volt_mode = 0;
            lock_at(a, CAL_I_LOW);
            f->int_d = c->v1;                 /* bumpless return to current control */
            if (++c->rep >= L_REPS) {
                if (c->tau_n < L_REPS) { fail(a); return true; }
                float L = c->tau_acc / (float)c->tau_n * p->R;
                if (!(L > 20e-6f && L < 10e-3f)) { fail(a); return true; }
                p->L = L;
                c->step = C_FWD;
                c->theta_cmd = 0.0f;
                c->count0 = a->enc.count;
                c->last_code = a->enc.hall_code;
            } else c->step = C_L_HOLD;
            c->t = 0;
        }
        break;
    case C_FWD:
    case C_REV: {
        lock_at(a, CAL_I_ALIGN);
        float dir = c->step == C_FWD ? 1.0f : -1.0f;
        c->theta_cmd += dir * F_2PI * CAL_SWEEP_HZ * TS;
        uint8_t code = a->enc.hall_code;
        if (code != c->last_code) {
            int slot = code_slot(c, code);
            if (slot < 0) { fail(a); return true; }
            /* only after the first electrical turn: the rotor has caught up by then */
            float turns = c->step == C_FWD ? c->theta_cmd / F_2PI : SWEEP_REVS - c->theta_cmd / F_2PI;
            if (turns > 1.0f) {
                if (c->step == C_FWD) { c->edge_fwd[slot] = wrap_pi(c->theta_cmd); c->seen_fwd |= (uint8_t)(1u << slot); }
                else { c->edge_rev[slot] = wrap_pi(c->theta_cmd); c->seen_rev |= (uint8_t)(1u << slot); }
            }
            c->last_code = code;
        }
        if (c->step == C_FWD && c->theta_cmd >= SWEEP_REVS * F_2PI) {
            c->count1 = a->enc.count;
            float cpe = (float)(c->count1 - c->count0) / SWEEP_REVS;   /* counts per electrical turn */
            float expect = (float)ENC_CPR / (float)POLE_PAIRS;
            if (absf(absf(cpe) - expect) > 0.05f * expect) { fail(a); return true; }
            if (cpe < 0.0f) p->enc_dir = (int8_t)-p->enc_dir;
            c->step = C_REV;
        } else if (c->step == C_REV && c->theta_cmd <= 0.0f) {
            c->theta_cmd = 0.0f;
            c->step = C_DONE;
            c->t = 0;
        }
        break;
    }
    case C_DONE:
    default:
        lock_at(a, CAL_I_ALIGN);
        if (c->t < T(200)) break;            /* settle at angle 0, then align the encoder here */
        finish_halls(a);
        if (a->state == ST_FAULT) return true;
        p->cal_valid = 1;
        params_apply_gains(a);
        a->faults &= (uint16_t)~F_NOT_CAL;
        /* the rotor sits at electrical angle 0: the encoder offset is known right now */
        enc_init(&a->enc, a->enc.last_raw, a->enc.hall_code);
        a->enc.sector = hall_sector(p, a->enc.hall_code);
        a->enc.aligned = 1;
        a->enc.offset_e = 0.0f;
        f->id_ref = 0.0f;
        return true;
    }
    return false;
}
