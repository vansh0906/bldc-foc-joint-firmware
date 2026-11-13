#ifndef SIM_H
#define SIM_H

#include "joint.h"
#include "plant.h"

#define SIM_MAX_J 8
#define LOG_COLS 20
/* log columns:
 *  0 t  1 pos_true  2 vel_true  3 pos_fw  4 vel_fw  5 iq_ref  6 iq_fw  7 iq_true  8 id_true
 *  9 torque  10 vbus  11 T_winding  12 T_ntc  13 T_winding_est  14 state  15 faults
 * 16 angle_error_e  17 angle_source  18 id_fw  19 ia_true */

struct sim;

typedef struct {
    struct sim *sim;
    int node;
    plant_t plant;
    app_t app;
    uint8_t flash[256];
    uint32_t tick;
    double last_kick, th_isr;
    int hang_ctrl, dead, regen_off;
    int wdg_resets, flash_writes, pwm_on_events;
    double *log;
    int log_n, log_cap, log_div;
} sjoint_t;

typedef struct { double t; int src; uint8_t b; int bad; } ev_t;
typedef struct { double t0, t1; int node; } txint_t;

typedef struct sim {
    int nj;
    sjoint_t j[SIM_MAX_J];
    double t, now_us;
    uint64_t steps;
    double baud, ber, wdg_timeout;
    rng_t rng;
    ev_t ev[8192];
    int nev, ev_dropped;
    txint_t tx[16];
    int ntx;
    uint8_t mrx[65536];
    double mrx_t[65536];
    int mrx_n;
    int collisions, late_tx, bit_errors;
    uint64_t tx_bytes;
    double bus_busy_until, bus_busy_us;
} sim_t;

void bus_tx(sim_t *s, int node, const uint8_t *buf, int n, double t_start_us);

sim_t *sim_new(int nj, uint64_t seed);
void sim_free(sim_t *s);
void sim_run(sim_t *s, double seconds);
void sim_master_send(sim_t *s, const uint8_t *buf, int n, double delay_us);
int sim_master_recv(sim_t *s, uint8_t *buf, double *t, int max);
void sim_local_cmd(sim_t *s, int j, int mode, double pos, double vel, double tau, double kp, double kd);
int sim_local(sim_t *s, int j, const char *what);
void sim_log_config(sim_t *s, int j, double rate_hz, int max_rows);
int sim_log_get(sim_t *s, int j, double *out, int max_rows);
double sim_time(sim_t *s);
int sim_set(sim_t *s, int j, const char *key, double v);
double sim_get(sim_t *s, int j, const char *key);

#endif
