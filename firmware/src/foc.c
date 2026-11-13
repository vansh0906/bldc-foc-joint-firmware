/* Field-oriented current control, run once per PWM period from the ADC interrupt. */
#include "joint.h"

void foc_reset(foc_t *f)
{
    f->id = f->iq = f->id_ref = f->iq_ref = 0.0f;
    f->int_d = f->int_q = 0.0f;
    f->vd = f->vq = 0.0f;
    f->duty[0] = f->duty[1] = f->duty[2] = 0.5f * DUTY_MAX;
    f->sat = 0;
    f->volt_mode = 0;
    f->vd_ol = f->vq_ol = 0.0f;
}

/* Pole-zero cancellation: PI zero at R/L, crossover at bw. */
void foc_set_gains(foc_t *f, float R, float L, float bw_hz)
{
    float wc = F_2PI * bw_hz;
    f->kp = L * wc;
    f->ki = R * wc;
    f->L = L;
    f->flux = MOTOR_FLUX;
}

void foc_measure(foc_t *f, const uint16_t raw[3], uint16_t vbus_raw)
{
    float i[3];
    for (int k = 0; k < 3; k++)
        i[k] = ((float)raw[k] - f->i_off[k]) * (1.0f / I_COUNTS_PER_A);
    /* the three phase currents sum to zero: remove whatever common-mode error is left */
    float cm = (i[0] + i[1] + i[2]) * (1.0f / 3.0f);
    f->ia = i[0] - cm;
    f->ib = i[1] - cm;
    f->ic = i[2] - cm;
    f->vbus = (float)vbus_raw * (1.0f / VBUS_COUNTS_PER_V);
}

void foc_dq(foc_t *f, float theta_e)
{
    float s, c;
    fsincos(theta_e, &s, &c);
    dq_t i = park(clarke(f->ia, f->ib, f->ic), s, c);
    f->id = i.d;
    f->iq = i.q;
}

void foc_current_loop(foc_t *f, float theta_e, float omega_e, float duty_out[3])
{
    float s, c;
    fsincos(theta_e, &s, &c);
    dq_t i = park(clarke(f->ia, f->ib, f->ic), s, c);
    f->id = i.d;
    f->iq = i.q;
    f->theta_e = theta_e;
    f->omega_e = omega_e;

    float vmax = V_MAX_FACTOR * f->vbus;
    dq_t v;
    if (f->volt_mode) {
        v.d = f->vd_ol;
        v.q = f->vq_ol;
        f->sat = (uint8_t)limit_dq(&v, vmax);
    } else {
        float ed = f->id_ref - i.d;
        float eq = f->iq_ref - i.q;
        v.d = f->kp * ed + f->int_d - omega_e * f->L * i.q;
        v.q = f->kp * eq + f->int_q + omega_e * (f->L * i.d + f->flux);
        f->sat = (uint8_t)limit_dq(&v, vmax);
        /* conditional integration: hold the integrator while saturated unless the error
         * would pull it back toward zero */
        if (!f->sat || ed * f->int_d < 0.0f) f->int_d += f->ki * TS * ed;
        if (!f->sat || eq * f->int_q < 0.0f) f->int_q += f->ki * TS * eq;
        f->int_d = clampf(f->int_d, -vmax, vmax);
        f->int_q = clampf(f->int_q, -vmax, vmax);
    }
    f->vd = v.d;
    f->vq = v.q;

    /* the new duties only reach the motor next period and act for a whole period: rotate the
     * voltage vector forward by the angle the rotor turns in that time */
    fsincos(theta_e + f->advance * TS * omega_e, &s, &c);
    svpwm(inv_park(v, s, c), f->vbus, DUTY_MAX, f->duty);
    duty_out[0] = f->duty[0];
    duty_out[1] = f->duty[1];
    duty_out[2] = f->duty[2];
}
