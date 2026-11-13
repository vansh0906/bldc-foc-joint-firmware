#include "plant.h"

#include <math.h>
#include <string.h>

#define CPR 4096

double rng_uniform(rng_t *r)
{
    r->s ^= r->s >> 12; r->s ^= r->s << 25; r->s ^= r->s >> 27;
    return (double)((r->s * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0);
}

double rng_gauss(rng_t *r)
{
    double u1 = rng_uniform(r), u2 = rng_uniform(r);
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static double sym(rng_t *r, double a) { return a * (2.0 * rng_uniform(r) - 1.0); }

void plant_init(plant_t *p, uint64_t seed, int randomize)
{
    memset(p, 0, sizeof *p);
    p->rng.s = seed * 0x9E3779B97F4A7C15ULL + 1;
    rng_t *r = &p->rng;
    double k = randomize ? 1.0 : 0.0;
    p->pp = 7;
    /* unit-to-unit spread around the datasheet values */
    p->R25 = 0.25 * (1.0 + k * sym(r, 0.08));
    p->Ld = 0.25e-3 * (1.0 + k * sym(r, 0.10));
    p->Lq = p->Ld * 1.05;               /* slight saliency */
    p->flux = 0.0085 * (1.0 + k * sym(r, 0.03));
    p->h6 = 0.015;
    p->J = 4.2e-5;                      /* rotor + output stage */
    p->B = 2e-5;
    p->Tc = 0.005;
    p->cog_amp = 0.002; p->cog_n = 84;  /* 12 slots x 14 poles */
    p->td = 200e-9; p->fpwm = 20000.0;
    p->Vs = 24.0; p->Rs = 0.05; p->Cbus = 470e-6; p->Rbleed = 10e3; p->supply_on = 1; p->supply_sink = 1;
    p->Tamb = 25.0; p->Rth_ws = 2.2; p->Cth_w = 28.0; p->Rth_sa = 3.0; p->Cth_s = 150.0; p->ntc_tau = 8.0;
    for (int i = 0; i < 3; i++) {
        p->adc_gain[i] = 1.0 + k * sym(r, 0.02);
        p->adc_off[i] = k * sym(r, 40.0);
        p->hall_shift[i] = k * sym(r, 4.0 * M_PI / 180.0);
    }
    p->adc_noise = 2.0;
    p->vbus_gain = 1.0 + k * sym(r, 0.01);
    p->hall_hyst = 1.0 * M_PI / 180.0;
    p->enc_mount = k * rng_uniform(r) * 2.0 * M_PI;
    p->enc_init = (int)(k * rng_uniform(r) * 60000.0);
    p->th = k * rng_uniform(r) * 2.0 * M_PI;      /* rotor stops wherever it stopped */
    p->vbus = p->Vs;
    p->Tw = p->Ts = p->Tntc = p->Tamb;
    for (int i = 0; i < 3; i++) p->duty[i] = p->duty_next[i] = 0.47;
    p->hall_state = plant_halls(p);
}

void plant_phase_currents(const plant_t *p, double i[3])
{
    double th = p->pp * p->th, c = cos(th), s = sin(th);
    double ia = c * p->id - s * p->iq, ib_ = s * p->id + c * p->iq;
    i[0] = ia;
    i[1] = -0.5 * ia + 0.5 * sqrt(3.0) * ib_;
    i[2] = -0.5 * ia - 0.5 * sqrt(3.0) * ib_;
}

static void derivs(const plant_t *p, double id, double iq, double th, double w,
                   double *did, double *diq, double *pole_out)
{
    double the = p->pp * th, we = p->pp * w;
    double c = cos(the), s = sin(the);
    double R = p->R25 * (1.0 + 0.0039 * (p->Tw - 25.0));
    double fl = p->flux * (1.0 + p->h6 * cos(6.0 * the));
    double vd = 0.0, vq = 0.0;
    if (p->pwm_on) {
        double ia = c * id - s * iq, ib_ = s * id + c * iq;
        double ip[3] = { ia, -0.5 * ia + 0.5 * sqrt(3.0) * ib_, -0.5 * ia - 0.5 * sqrt(3.0) * ib_ };
        double v[3];
        for (int k = 0; k < 3; k++) {
            /* dead time: during the blanking both switches are off and the diode that conducts
             * depends on the current sign, so the pole loses td/Ts of the bus in the current's direction */
            double dt_err = p->td * p->fpwm * p->vbus * tanh(ip[k] / 0.3);
            v[k] = p->duty[k] * p->vbus - dt_err;
            if (p->short_phase == k + 1) v[k] = 0.0;
            if (pole_out) pole_out[k] = v[k];
        }
        double va = (2.0 * v[0] - v[1] - v[2]) / 3.0, vb = (v[1] - v[2]) / sqrt(3.0);
        vd = c * va + s * vb;
        vq = -s * va + c * vb;
        *did = (vd - R * id + we * p->Lq * iq) / p->Ld;
        *diq = (vq - R * iq - we * p->Ld * id - we * fl) / p->Lq;
        if (p->short_phase) {      /* a shorted low side drives the current up hard */
            *did += 2e4 * (id >= 0 ? 1.0 : -1.0);
            *diq += 2e4 * (iq >= 0 ? 1.0 : -1.0);
        }
    } else {
        /* all switches off: the winding current freewheels through the diodes into the bus
         * and dies out in a few tens of microseconds (back-EMF stays below the bus voltage) */
        *did = -id / 20e-6;
        *diq = -iq / 20e-6;
        if (pole_out) pole_out[0] = pole_out[1] = pole_out[2] = 0.0;
    }
}

void plant_step(plant_t *p, double dt)
{
    const int NSUB = 4;
    double h = dt / NSUB;
    double i_dc_acc = 0.0;
    for (int n = 0; n < NSUB; n++) {
        double did1, diq1, did2, diq2, pole[3];
        derivs(p, p->id, p->iq, p->th, p->w, &did1, &diq1, pole);
        double idm = p->id + 0.5 * h * did1, iqm = p->iq + 0.5 * h * diq1;
        derivs(p, idm, iqm, p->th + 0.5 * h * p->w, p->w, &did2, &diq2, NULL);
        p->id += h * did2;
        p->iq += h * diq2;
        if (!p->pwm_on && fabs(p->id) < 1e-6 && fabs(p->iq) < 1e-6) p->id = p->iq = 0.0;

        double the = p->pp * p->th;
        double fl = p->flux * (1.0 + p->h6 * cos(6.0 * the));
        p->Te = 1.5 * p->pp * (fl * p->iq + (p->Ld - p->Lq) * p->id * p->iq);
        double Tload = p->T_load + p->grav * sin(p->th) + p->spring_k * (p->th - p->spring_x0)
                     + p->cog_amp * sin(p->cog_n * p->th) + p->B * p->w + p->Tc * tanh(p->w / 0.02);
        if (p->locked) p->w = 0.0;
        else {
            p->w += h * (p->Te - Tload) / p->J;
            p->th += h * p->w;
        }

        double ip[3];
        plant_phase_currents(p, ip);
        double i_dc = p->pwm_on ? (p->duty[0] * ip[0] + p->duty[1] * ip[1] + p->duty[2] * ip[2]) : 0.0;
        if (!p->pwm_on) {        /* freewheeling energy returns to the bus */
            double we = p->pp * p->w;
            i_dc = -1.5 * (we * p->flux * p->iq) / (p->vbus > 1.0 ? p->vbus : 1.0);
        }
        /* the supply holds the bus through Rs; a bench supply cannot take current back */
        if (p->supply_on && (p->supply_sink || p->Vs > p->vbus)) {
            double tau = p->Rs * p->Cbus;
            double vinf = p->Vs - (i_dc + p->vbus / p->Rbleed) * p->Rs;
            double vn = vinf + (p->vbus - vinf) * exp(-h / tau);
            if (!p->supply_sink && vn > p->Vs) vn = p->vbus + h * (-i_dc - p->vbus / p->Rbleed) / p->Cbus;
            p->vbus = vn;
        } else {
            p->vbus += h * (-i_dc - p->vbus / p->Rbleed) / p->Cbus;
        }
        if (p->vbus < 0.0) p->vbus = 0.0;
        i_dc_acc += i_dc;
        (void)pole;
    }
    (void)i_dc_acc;
    double ip[3];
    plant_phase_currents(p, ip);
    p->ia = ip[0]; p->ib = ip[1]; p->ic = ip[2];

    /* thermal: winding -> stator -> ambient, NTC glued to the stator */
    double R = p->R25 * (1.0 + 0.0039 * (p->Tw - 25.0));
    double P = 1.5 * R * (p->id * p->id + p->iq * p->iq);
    double q_ws = (p->Tw - p->Ts) / p->Rth_ws;
    p->Tw += dt * (P - q_ws) / p->Cth_w;
    p->Ts += dt * (q_ws - (p->Ts - p->Tamb) / p->Rth_sa) / p->Cth_s;
    p->Tntc += dt * (p->Ts - p->Tntc) / p->ntc_tau;

    p->t += dt;
    if (p->have_next) {
        memcpy(p->duty, p->duty_next, sizeof p->duty);
        p->have_next = 0;
    }
}

void plant_adc(plant_t *p, uint16_t i_raw[3], uint16_t *vbus_raw)
{
    double ip[3];
    plant_phase_currents(p, ip);
    for (int k = 0; k < 3; k++) {
        double c = 2048.0 + p->adc_off[k] + ip[k] * 81.92 * p->adc_gain[k] + p->adc_noise * rng_gauss(&p->rng);
        c = floor(c + 0.5);
        i_raw[k] = (uint16_t)(c < 0 ? 0 : (c > 4095 ? 4095 : c));
    }
    double v = p->vbus * 65.28 * p->vbus_gain + 1.0 * rng_gauss(&p->rng);
    v = floor(v + 0.5);
    *vbus_raw = (uint16_t)(v < 0 ? 0 : (v > 4095 ? 4095 : v));
}

uint16_t plant_ntc_raw(plant_t *p)
{
    double R = 10000.0 * exp(3435.0 * (1.0 / (p->Tntc + 273.15) - 1.0 / 298.15));
    double c = 4096.0 * R / (R + 10000.0) + 1.0 * rng_gauss(&p->rng);
    return (uint16_t)(c < 0 ? 0 : (c > 4095 ? 4095 : c));
}

uint16_t plant_encoder(plant_t *p)
{
    if (p->enc_frozen) return p->enc_frozen_val;
    double c = floor((p->th + p->enc_mount) * CPR / (2.0 * M_PI));
    if (p->enc_rev) c = -c;
    int64_t n = (int64_t)c + p->enc_init + p->enc_slip;
    uint16_t v = (uint16_t)(n & 0xFFFF);
    p->enc_frozen_val = v;
    return v;
}

uint8_t plant_halls(plant_t *p)
{
    if (p->halls_dead) return 7;
    double the = p->pp * p->th;
    double h = sin(p->hall_hyst / 2.0);
    uint8_t out = 0;
    for (int k = 0; k < 3; k++) {
        double s = sin(the - k * 2.0 * M_PI / 3.0 - p->hall_shift[k]);
        int prev = (p->hall_state >> k) & 1;
        int now = s > h ? 1 : (s < -h ? 0 : prev);
        out |= (uint8_t)(now << k);
    }
    p->hall_state = out;
    return out;
}
