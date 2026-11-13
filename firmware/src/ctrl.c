/* Outer loops at 1 kHz. Every mode ends in a torque request; the app turns it into an iq
 * reference for the current loop after the limits.
 *
 *   torque     tau = tau_ff
 *   velocity   PI on speed (+ tau_ff)
 *   position   P on position -> speed reference (+ vel_ref feed-forward) -> velocity PI
 *   impedance  tau = kp (pos_ref - pos) + kd (vel_ref - vel) + tau_ff    (legged/humanoid style) */
#include "joint.h"

void ctrl_reset(ctrl_t *c)
{
    c->mode = MODE_TORQUE;
    c->pos_ref = c->vel_ref = c->tau_ff = c->kp = c->kd = 0.0f;
    c->vel_int = 0.0f;
    c->tau_cmd = c->iq_cmd = 0.0f;
    c->i_lim_eff = I_MAX;
}

static float velocity_pi(app_t *a, float vref, float tau_max)
{
    ctrl_t *c = &a->ctrl;
    const params_t *p = &a->prm;
    float e = vref - a->enc.vel;
    float tau = p->vel_kp * e + c->vel_int + c->tau_ff;
    /* integrate only while the output is inside the torque limit (or the error unwinds it), and
     * not while the ISR is cutting braking current to protect the bus */
    bool regen = a->regen_hits != a->regen_seen;
    if (((tau < tau_max && tau > -tau_max) || e * c->vel_int < 0.0f) && !regen)
        c->vel_int += p->vel_ki * e * CTRL_DT;
    c->vel_int = clampf(c->vel_int, -tau_max, tau_max);
    return tau;
}

float ctrl_torque(app_t *a)
{
    ctrl_t *c = &a->ctrl;
    const params_t *p = &a->prm;
    float tau_max = MOTOR_KT * c->i_lim_eff;
    float tau;
    switch (c->mode) {
    case MODE_VELOCITY:
        tau = velocity_pi(a, clampf(c->vel_ref, -p->vel_limit, p->vel_limit), tau_max);
        break;
    case MODE_POSITION: {
        float vref = p->pos_kp * (c->pos_ref - a->enc.pos) + c->vel_ref;
        tau = velocity_pi(a, clampf(vref, -p->vel_limit, p->vel_limit), tau_max);
        break;
    }
    case MODE_IMPEDANCE:
        tau = c->kp * (c->pos_ref - a->enc.pos) + c->kd * (c->vel_ref - a->enc.vel) + c->tau_ff;
        break;
    case MODE_TORQUE:
    default:
        tau = c->tau_ff;
        break;
    }
    tau = clampf(tau, -tau_max, tau_max);
    c->tau_cmd = tau;
    return tau;
}
