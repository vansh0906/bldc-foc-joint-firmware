/* Protections that are slower than one PWM period. Overcurrent is in the ISR (app.c) and, on
 * the target, also in hardware (TIM1 break input from the comparators).
 *
 * Thermal: the NTC sits on the stator and lags the copper by tens of seconds, so the current
 * limit is based on an estimated winding temperature: NTC + a first-order model of the
 * winding-to-stator rise driven by the measured copper loss. */
#include "joint.h"

#define UV_DEBOUNCE_MS  10u
#define OV_DEBOUNCE_MS  2u
#define CU_TEMPCO       0.0039f

static const float NTC_RAW[18] = {
    3628.0f, 3368.3f, 3037.7f, 2654.3f, 2248.6f, 1854.0f, 1496.8f, 1191.3f, 940.6f,
    740.6f, 583.9f, 462.2f, 368.0f, 295.0f, 238.4f, 194.1f, 159.4f, 131.9f
};   /* 10k NTC, B = 3435, 10k pull-up: ADC counts at -20, -10, ... 150 degC */

float ntc_raw_to_c(uint16_t raw)
{
    float r = (float)raw;
    if (r >= NTC_RAW[0]) return -20.0f;
    for (int k = 1; k < 18; k++) {
        if (r >= NTC_RAW[k]) {
            float f = (NTC_RAW[k - 1] - r) / (NTC_RAW[k - 1] - NTC_RAW[k]);
            return -20.0f + 10.0f * ((float)(k - 1) + f);
        }
    }
    return 150.0f;
}

void prot_reset(prot_t *p)
{
    p->vbus_f = 0.0f;
    p->ov_cnt = p->uv_cnt = 0;
    p->t_ntc = 25.0f;
    p->t_wind_est = 25.0f;
    p->dT_model = 0.0f;
    p->loss_acc = 0.0f;
    p->loss_n = 0;
    p->derate = 1.0f;
}

void prot_fast(app_t *a)
{
    prot_t *p = &a->prot;
    if (p->vbus_f > VBUS_OV_TRIP) {
        if (++p->ov_cnt >= OV_DEBOUNCE_MS) app_trip(a, F_OVERVOLT);
    } else p->ov_cnt = 0;
    if (p->vbus_f < VBUS_UV_TRIP) {
        if (++p->uv_cnt >= UV_DEBOUNCE_MS) app_trip(a, F_UNDERVOLT);
    } else p->uv_cnt = 0;

    float lim = a->prm.i_limit * p->derate;
    if (a->enc.fault) lim *= 0.5f;      /* hall-only commutation: keep it gentle */
    a->ctrl.i_lim_eff = lim;
}

void prot_slow(app_t *a)
{
    prot_t *p = &a->prot;
    p->t_ntc = ntc_raw_to_c(a->hal.ops->read_ntc(a->hal.ctx));

    float loss = p->loss_n ? p->loss_acc / (float)p->loss_n : 0.0f;
    p->loss_acc = 0.0f;
    p->loss_n = 0;
    float dt = CTRL_DT * (float)MON_DIV;
    p->dT_model += (loss - p->dT_model / TH_RTH_W_S) / TH_CTH_W * dt;
    p->t_wind_est = p->t_ntc + p->dT_model;

    float t_eff = p->ntc_only ? p->t_ntc : p->t_wind_est;
    p->derate = clampf((T_DERATE_END - t_eff) / (T_DERATE_END - T_DERATE_START), 0.0f, 1.0f);
    if (p->derate < 1.0f) a->faults |= F_DERATING;
    else a->faults &= (uint16_t)~F_DERATING;

    if (p->t_ntc > T_TRIP_NTC || (!p->ntc_only && p->t_wind_est > T_TRIP_WINDING))
        app_trip(a, F_OVERTEMP);
}

/* copper loss for the thermal model, called every control tick */
void prot_account_loss(app_t *a, float id, float iq)
{
    prot_t *p = &a->prot;
    float r = a->prm.R * (1.0f + CU_TEMPCO * (p->t_wind_est - 25.0f));
    p->loss_acc += 1.5f * r * (id * id + iq * iq);
    p->loss_n++;
}

float prot_limit_iq(app_t *a, float iq)
{
    /* the braking-current limit itself runs in the ISR (app.c); this only reports it */
    if (a->regen_hits != a->regen_seen) a->faults |= F_REGEN_LIMIT;
    else a->faults &= (uint16_t)~F_REGEN_LIMIT;
    a->regen_seen = a->regen_hits;
    float lim = a->ctrl.i_lim_eff;
    return clampf(iq, -lim, lim);
}
