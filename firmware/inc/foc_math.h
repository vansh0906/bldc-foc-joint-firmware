/* Math used by the current loop. No libm: the firmware builds freestanding for Cortex-M4F,
 * where sqrtf maps to the VSQRT instruction and sin/cos are short polynomials. */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>

#define F_PI     3.14159265f
#define F_2PI    6.28318531f
#define F_SQRT3  1.73205081f

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float absf(float x) { return x < 0.0f ? -x : x; }
static inline float fm_sqrt(float x) { return x > 0.0f ? __builtin_sqrtf(x) : 0.0f; }

float wrap_pi(float x);          /* -> [-pi, pi) */
float wrap_2pi(float x);         /* -> [0, 2pi) */
void fsincos(float x, float *s, float *c);   /* max error 4e-6 on any input */

typedef struct { float a, b; } ab_t;     /* stationary alpha/beta */
typedef struct { float d, q; } dq_t;     /* rotor frame */

ab_t clarke(float ia, float ib, float ic);            /* amplitude invariant */
dq_t park(ab_t x, float s, float c);
ab_t inv_park(dq_t x, float s, float c);

/* Space-vector PWM by min/max zero-sequence injection. Returns duties in [0, DUTY_MAX];
 * returns 1 if the request had to be clipped (outside the hexagon). */
int svpwm(ab_t v, float vbus, float duty_max, float duty[3]);

/* Voltage vector limit with d-axis priority. Returns 1 if anything was limited. */
int limit_dq(dq_t *v, float vmax);

#endif
