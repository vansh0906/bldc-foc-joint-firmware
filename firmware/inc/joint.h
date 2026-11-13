/* Joint controller: all state lives in one app_t so several joints can be simulated in one
 * process and every module can be unit tested on its own. On the target there is one app_t. */
#ifndef JOINT_H
#define JOINT_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "foc_math.h"
#include "hal.h"

typedef enum { ST_BOOT = 0, ST_IDLE = 1, ST_RUN = 2, ST_DAMP = 3, ST_CAL = 4, ST_FAULT = 5 } jstate_t;
typedef enum { MODE_TORQUE = 0, MODE_VELOCITY = 1, MODE_POSITION = 2, MODE_IMPEDANCE = 3 } jmode_t;
typedef enum { ANG_NONE = 0, ANG_HALL = 1, ANG_ENCODER = 2, ANG_FORCED = 3, ANG_SIXSTEP = 4 } angsrc_t;

/* fault and warning bits (status word sent on the bus) */
#define F_OVERCURRENT   0x0001u   /* latched */
#define F_OVERVOLT      0x0002u   /* latched */
#define F_UNDERVOLT     0x0004u   /* latched */
#define F_OVERTEMP      0x0008u   /* latched */
#define F_CTRL_STALL    0x0010u   /* latched: control task stopped running */
#define F_CAL_FAILED    0x0020u   /* latched */
#define F_ADC_OFFSET    0x0040u   /* latched: current sensor offset out of range at boot */
#define F_ENCODER       0x0100u   /* warning: running on hall sensors at reduced current */
#define F_COMM_TIMEOUT  0x0200u   /* warning: in damping mode until cleared */
#define F_DERATING      0x0400u   /* warning: thermal current derating active */
#define F_REGEN_LIMIT   0x0800u   /* warning: braking current reduced (bus voltage high) */
#define F_NOT_CAL       0x1000u   /* info: no valid calibration stored */
#define F_PARAMS_RESET  0x2000u   /* info: stored parameters were invalid, defaults loaded */
#define F_LATCHED_MASK  0x00FFu

#define PARAMS_MAGIC    0x4A425553u  /* "JBUS" */
#define PARAMS_VERSION  3u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t node_id;
    uint8_t cal_valid;
    float R, L;                  /* measured phase resistance and inductance */
    float hall_b[6];             /* electrical angle where sector k starts (rad) */
    uint8_t code_of_sector[6];   /* hall code seen in sector k */
    int8_t enc_dir;              /* +1/-1: encoder counts vs electrical angle */
    uint8_t pad0;
    float cur_bw_hz;
    float vel_kp, vel_ki, pos_kp;
    float i_limit, vel_limit;
    float kd_safe;
    uint32_t baud;
    uint16_t comm_timeout_ms;
    uint16_t crc;                /* CRC-16/MODBUS over everything above */
} params_t;

typedef struct {
    float i_off[3];              /* ADC offsets in counts, measured at boot */
    float ia, ib, ic, vbus;
    float id, iq, id_ref, iq_ref;
    float int_d, int_q, kp, ki;
    float L, flux;               /* for the decoupling feed-forward */
    float vd, vq;
    float duty[3];
    float theta_e, omega_e;      /* angle and speed the loop used this period */
    uint8_t sat;
    uint8_t volt_mode;           /* calibration: open-loop voltage instead of current control */
    float vd_ol, vq_ol;
    float advance;               /* angle advance factor (periods) for the PWM update delay */
} foc_t;

typedef struct {
    uint16_t last_raw;
    int32_t count;               /* accumulated encoder counts, direction corrected */
    int32_t pll_base;            /* PLL estimate = pll_base + pll_off (counts) */
    float pll_off, pll_vel;      /* counts, counts/s */
    float kp_pll, ki_pll;
    uint8_t hall_code;
    int8_t sector, hall_dir;
    uint8_t hall_ok;
    uint32_t t_edge, dt_edge;    /* ISR ticks */
    float hall_theta0, hall_omega_e;
    int32_t hall_steps;          /* signed count of sector changes (position on halls) */
    uint8_t aligned;
    float offset_e;              /* electrical angle = enc_angle - offset_e */
    int32_t count_at_edge;
    uint8_t bad_edges, frozen_edges, fault;
    angsrc_t src, src_override;
    float theta_e, omega_e;      /* electrical angle/speed from the active source */
    float pos, vel;              /* mechanical rad, rad/s from the active source */
    float pos_hall_offset;       /* keeps position continuous when falling back to halls */
    uint32_t edges;
} enc_t;

typedef struct {
    jmode_t mode;
    float pos_ref, vel_ref, tau_ff, kp, kd;
    float vel_int;
    float tau_cmd, iq_cmd;
    float i_lim_eff;
    uint32_t ms, last_cmd_ms;
    uint8_t cmd_seen;
    uint8_t mon_div;
    uint32_t damp_ms;
} ctrl_t;

typedef struct {
    float vbus_f;
    uint8_t ov_cnt, uv_cnt;
    float t_ntc, t_wind_est, dT_model;
    float loss_acc;              /* copper loss accumulated over the monitor period */
    uint32_t loss_n;
    float derate;
    uint8_t ntc_only;            /* test switch: derate on the NTC alone (no winding model) */
    uint8_t regen_off;           /* test switch: no braking-current limit */
} prot_t;

typedef struct {
    uint8_t step;
    uint32_t t;                  /* ISR ticks in this step */
    float acc_v, acc_i;
    uint32_t acc_n;
    float v1, i1, v2, i2;
    float tau_acc; uint8_t tau_n;
    float theta_cmd;
    float edge_fwd[6], edge_rev[6];
    uint8_t seen_fwd, seen_rev;
    uint8_t last_code;
    int32_t count0, count1;
    uint8_t codes[6]; uint8_t ncodes;
    uint8_t rep;
} cal_t;

#define JBUS_MAX_FRAME  128
#define JBUS_RING       256

typedef struct {
    uint16_t ring[JBUS_RING];    /* bytes; 0x100 marks an idle gap (resync point) */
    uint32_t ring_t[JBUS_RING];  /* receive time of each entry (us) */
    volatile uint16_t head, tail;
    uint8_t buf[JBUS_MAX_FRAME];
    uint16_t n, need;
    uint8_t st;
    uint32_t last_rx_us;
    uint8_t tx[JBUS_MAX_FRAME];
    /* statistics */
    uint32_t frames_ok, crc_err, len_err, gap_resync, replies, overflow;
} jbus_t;

typedef struct {
    hal_t hal;
    params_t prm;
    foc_t foc;
    enc_t enc;
    ctrl_t ctrl;
    prot_t prot;
    cal_t cal;
    jbus_t bus;
    volatile jstate_t state;
    volatile uint16_t faults;
    uint32_t isr_count;
    volatile uint32_t ctrl_stamp;    /* isr_count at the last control tick */
    volatile float iq_ref_shared;    /* control task -> ISR */
    float id_ref_shared;
    uint16_t boot_n;
    float boot_acc[3];
    uint8_t alive;                   /* task-alive bits for the watchdog */
    uint32_t wdg_kicks;
    float vbus_acc; uint16_t vbus_n;
    uint32_t regen_hits, regen_seen; /* ISR periods with braking current reduced */
    uint8_t no_preload;              /* test switch: enable without the back-EMF preload */
} app_t;

/* ---- app.c ---- */
void app_init(app_t *a, hal_t hal);
void app_isr(app_t *a);              /* PWM-synchronous, 20 kHz */
void app_ctrl_tick(app_t *a);        /* 1 kHz control task */
void app_monitor(app_t *a);          /* 100 Hz: temperature, watchdog */
void app_uart_rx(app_t *a, uint8_t byte, uint32_t t_us);   /* UART RX interrupt */
void app_comm_poll(app_t *a);        /* comm task: parse and answer frames */
bool app_enable(app_t *a);
void app_disable(app_t *a);
bool app_clear_faults(app_t *a);
bool app_start_cal(app_t *a);
void app_set_command(app_t *a, jmode_t mode, float pos, float vel, float tau_ff, float kp, float kd);
void app_trip(app_t *a, uint16_t fault);

/* ---- params.c ---- */
void params_defaults(params_t *p);
uint16_t params_crc(const params_t *p);
bool params_load(app_t *a);
bool params_save(app_t *a);
void params_apply_gains(app_t *a);

/* ---- foc.c ---- */
void foc_reset(foc_t *f);
void foc_set_gains(foc_t *f, float R, float L, float bw_hz);
void foc_measure(foc_t *f, const uint16_t raw[3], uint16_t vbus_raw);
void foc_dq(foc_t *f, float theta_e);   /* id/iq of this period's sample */
void foc_current_loop(foc_t *f, float theta_e, float omega_e, float duty_out[3]);

/* ---- encoder.c ---- */
void enc_init(enc_t *e, uint16_t raw, uint8_t halls);
void enc_update(app_t *a, uint16_t raw, uint8_t halls);
int8_t hall_sector(const params_t *p, uint8_t code);

/* ---- ctrl.c ---- */
void ctrl_reset(ctrl_t *c);
float ctrl_torque(app_t *a);        /* runs the selected mode, returns the torque request */

/* ---- protect.c ---- */
void prot_reset(prot_t *p);
void prot_fast(app_t *a);           /* 1 kHz: bus voltage */
void prot_slow(app_t *a);           /* 100 Hz: temperature */
float prot_limit_iq(app_t *a, float iq);   /* derating and regen limit */
void prot_account_loss(app_t *a, float id, float iq);
float ntc_raw_to_c(uint16_t raw);

/* ---- calib.c ---- */
void cal_start(app_t *a);
bool cal_isr(app_t *a);              /* returns true when finished (state left) */

/* ---- jbus.c ---- */
uint16_t crc16_modbus(const uint8_t *d, uint16_t n);
void jbus_reset(jbus_t *b);
void jbus_poll(app_t *a);
uint16_t jbus_build_status(app_t *a, uint8_t instr, uint8_t *out);

#endif
