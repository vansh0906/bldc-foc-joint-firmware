/* Parameters kept in flash: node id, calibration results and tuning. Protected by a CRC so a
 * half-written or blank page loads safe defaults instead of garbage. */
#include <stddef.h>
#include <string.h>

#include "joint.h"

void params_defaults(params_t *p)
{
    memset(p, 0, sizeof *p);
    p->magic = PARAMS_MAGIC;
    p->version = PARAMS_VERSION;
    p->node_id = JBUS_DEFAULT_ID;
    p->cal_valid = 0;
    p->R = MOTOR_R_NOM;
    p->L = MOTOR_L_NOM;
    /* standard 120-degree hall placement; calibration measures the real boundaries */
    static const uint8_t codes[6] = { 5, 1, 3, 2, 6, 4 };
    for (int k = 0; k < 6; k++) {
        p->hall_b[k] = wrap_pi((float)k * (F_PI / 3.0f));
        p->code_of_sector[k] = codes[k];
    }
    p->enc_dir = 1;
    p->cur_bw_hz = I_LOOP_BW_HZ;
    p->vel_kp = 0.010f;      /* Nm per rad/s: ~38 Hz speed loop on the bare joint */
    p->vel_ki = 0.3f;        /* Nm per rad */
    p->pos_kp = 80.0f;       /* (rad/s) per rad: ~13 Hz position loop */
    p->i_limit = I_MAX;
    p->vel_limit = VEL_LIMIT;
    p->kd_safe = KD_SAFE;
    p->baud = JBUS_BAUD;
    p->comm_timeout_ms = COMM_TIMEOUT_MS;
    p->crc = params_crc(p);
}

uint16_t params_crc(const params_t *p)
{
    return crc16_modbus((const uint8_t *)p, (uint16_t)offsetof(params_t, crc));
}

bool params_load(app_t *a)
{
    params_t tmp;
    bool ok = a->hal.ops->flash_read(a->hal.ctx, &tmp, (uint16_t)sizeof tmp)
              && tmp.magic == PARAMS_MAGIC && tmp.version == PARAMS_VERSION
              && tmp.crc == params_crc(&tmp);
    if (ok) a->prm = tmp;
    else {
        params_defaults(&a->prm);
        a->faults |= F_PARAMS_RESET;
    }
    if (!a->prm.cal_valid) a->faults |= F_NOT_CAL;
    return ok;
}

bool params_save(app_t *a)
{
    a->prm.crc = params_crc(&a->prm);
    return a->hal.ops->flash_write(a->hal.ctx, &a->prm, (uint16_t)sizeof a->prm);
}

void params_apply_gains(app_t *a)
{
    foc_set_gains(&a->foc, a->prm.R, a->prm.L, a->prm.cur_bw_hz);
}
