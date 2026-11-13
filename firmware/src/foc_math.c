#include "foc_math.h"

float wrap_pi(float x)
{
    /* inputs are bounded in practice (angles, angle sums); the cast keeps it libm-free */
    float k = x * (1.0f / F_2PI);
    int32_t n = (int32_t)(k >= 0.0f ? k + 0.5f : k - 0.5f);
    x -= (float)n * F_2PI;
    if (x >= F_PI) x -= F_2PI;
    if (x < -F_PI) x += F_2PI;
    return x;
}

float wrap_2pi(float x)
{
    x = wrap_pi(x);
    return x < 0.0f ? x + F_2PI : x;
}

/* odd polynomial for sin on [-pi/2, pi/2] (Taylor to x^11) */
static float sin_core(float x)
{
    float x2 = x * x;
    return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f
               + x2 * (1.0f / 362880.0f + x2 * (-1.0f / 39916800.0f))))));
}

static float fsin_r(float x)   /* x in [-pi, pi) */
{
    if (x > 0.5f * F_PI) x = F_PI - x;
    else if (x < -0.5f * F_PI) x = -F_PI - x;
    return sin_core(x);
}

void fsincos(float x, float *s, float *c)
{
    x = wrap_pi(x);
    *s = fsin_r(x);
    *c = fsin_r(wrap_pi(x + 0.5f * F_PI));
}

ab_t clarke(float ia, float ib, float ic)
{
    ab_t r;
    r.a = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
    r.b = (ib - ic) * (1.0f / F_SQRT3);
    return r;
}

dq_t park(ab_t x, float s, float c)
{
    dq_t r;
    r.d = c * x.a + s * x.b;
    r.q = -s * x.a + c * x.b;
    return r;
}

ab_t inv_park(dq_t x, float s, float c)
{
    ab_t r;
    r.a = c * x.d - s * x.q;
    r.b = s * x.d + c * x.q;
    return r;
}

int svpwm(ab_t v, float vbus, float duty_max, float duty[3])
{
    float va = v.a;
    float vb = -0.5f * v.a + 0.5f * F_SQRT3 * v.b;
    float vc = -0.5f * v.a - 0.5f * F_SQRT3 * v.b;
    float mx = va > vb ? (va > vc ? va : vc) : (vb > vc ? vb : vc);
    float mn = va < vb ? (va < vc ? va : vc) : (vb < vc ? vb : vc);
    float off = -0.5f * (mx + mn);
    float inv = vbus > 1.0f ? 1.0f / vbus : 1.0f;
    float mid = 0.5f * duty_max;
    int clipped = 0;
    float ph[3] = { va, vb, vc };
    for (int i = 0; i < 3; i++) {
        float d = mid + (ph[i] + off) * inv;
        if (d < 0.0f) { d = 0.0f; clipped = 1; }
        if (d > duty_max) { d = duty_max; clipped = 1; }
        duty[i] = d;
    }
    return clipped;
}

int limit_dq(dq_t *v, float vmax)
{
    int lim = 0;
    if (v->d > vmax) { v->d = vmax; lim = 1; }
    if (v->d < -vmax) { v->d = -vmax; lim = 1; }
    float qmax = fm_sqrt(vmax * vmax - v->d * v->d);
    if (v->q > qmax) { v->q = qmax; lim = 1; }
    if (v->q < -qmax) { v->q = -qmax; lim = 1; }
    return lim;
}
