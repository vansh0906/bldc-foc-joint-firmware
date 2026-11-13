/* Model of one joint's hardware: PMSM + three-phase inverter + DC bus + sensors + thermal.
 * This is test equipment, not firmware: it uses double precision and libm freely. */
#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { uint64_t s; } rng_t;
double rng_uniform(rng_t *r);           /* [0, 1) */
double rng_gauss(rng_t *r);

typedef struct {
    /* motor (true values; the firmware only knows nominal ones until it calibrates) */
    int pp;
    double R25, Ld, Lq, flux, h6;      /* h6: 6th-harmonic torque/back-EMF ripple ratio */
    double J, B, Tc, cog_amp; int cog_n;
    /* inverter, supply, bus */
    double td, fpwm;                    /* dead time, PWM frequency */
    double Vs, Rs, Cbus, Rbleed; int supply_on, supply_sink;   /* sink: a battery; 0: bench supply */
    int enc_rev;                        /* encoder A/B swapped */
    /* thermal */
    double Tamb, Rth_ws, Cth_w, Rth_sa, Cth_s, ntc_tau;
    /* sensor errors */
    double adc_gain[3], adc_off[3], adc_noise, vbus_gain;
    double hall_shift[3], hall_hyst;
    double enc_mount; int enc_init;
    /* loads */
    double T_load, grav, spring_k, spring_x0; int locked;
    /* injected faults */
    int enc_frozen; uint16_t enc_frozen_val; int enc_slip;
    int short_phase;                    /* 1..3: that phase's low-side shorted -> overcurrent */
    int halls_dead;                     /* hall connector unplugged: reads 0b111 (pull-ups) */
    /* state */
    double t, id, iq, th, w, vbus, Tw, Ts, Tntc;
    double duty[3], duty_next[3]; int pwm_on, have_next;
    double Te, ia, ib, ic;
    uint8_t hall_state;
    rng_t rng;
} plant_t;

void plant_init(plant_t *p, uint64_t seed, int randomize);
void plant_step(plant_t *p, double dt);       /* one PWM period */
void plant_phase_currents(const plant_t *p, double i[3]);
/* sensor reads as the firmware's ADC / timers would see them */
void plant_adc(plant_t *p, uint16_t i_raw[3], uint16_t *vbus_raw);
uint16_t plant_ntc_raw(plant_t *p);
uint16_t plant_encoder(plant_t *p);
uint8_t plant_halls(plant_t *p);

#endif
