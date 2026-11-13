/* Build-time configuration of the joint controller.
 *
 * Board: STM32G431 (Cortex-M4F, 170 MHz) three-phase inverter with low-side shunts,
 * e.g. ST's B-G431B-ESC1, plus an RS-485 transceiver for the joint bus.
 * Motor: 7 pole-pair outrunner BLDC/PMSM used as a robot joint actuator.
 * Values marked (nominal) are only starting points: calibration measures R and L and the hall
 * sensor positions on each unit. */
#ifndef CONFIG_H
#define CONFIG_H

#define FW_VERSION          0x0100u      /* 1.0 */
#define MODEL_ID            0x4A38u      /* "J8" */

/* ---- timing ---- */
#define PWM_HZ              20000.0f     /* center-aligned PWM, current loop runs every period */
#define TS                  (1.0f / PWM_HZ)
#define CTRL_DIV            20u          /* outer loops at PWM_HZ / CTRL_DIV = 1 kHz */
#define CTRL_DT             (TS * (float)CTRL_DIV)
#define MON_DIV             10u          /* monitor at 100 Hz (every 10th control tick) */

/* ---- motor (nominal) ---- */
#define POLE_PAIRS          7
#define MOTOR_R_NOM         0.25f        /* ohm, phase */
#define MOTOR_L_NOM         0.00025f     /* H, phase (Ld = Lq) */
#define MOTOR_FLUX          0.0085f      /* Wb, permanent-magnet flux linkage */
#define MOTOR_KT            (1.5f * (float)POLE_PAIRS * MOTOR_FLUX)   /* Nm per A of iq */

/* ---- sensors ---- */
#define ENC_CPR             4096         /* incremental ABZ encoder, counts per mechanical rev */
#define ADC_FULL            4096.0f
#define ADC_MID             2048.0f
#define I_COUNTS_PER_A      81.92f       /* +-25 A across the ADC range (shunt + amplifier gain) */
#define VBUS_COUNTS_PER_V   65.28f       /* 1:19 divider into 3.3 V */

/* ---- limits ---- */
#define I_MAX               10.0f        /* A, peak dq current command */
#define I_OC_TRIP           15.0f        /* A, any phase: PWM off within one period */
#define VBUS_OV_TRIP        30.0f        /* V */
#define VBUS_UV_TRIP        18.0f        /* V */
#define VBUS_REGEN_START    26.0f        /* V, braking current is reduced above this ... */
#define VBUS_REGEN_ZERO     28.5f        /* ... and is zero here (1.5 V below the trip) */
#define DUTY_MAX            0.94f        /* keeps the low-side switch on long enough to sample the shunts */
#define V_MAX_FACTOR        (0.92f * DUTY_MAX / 1.7320508f)   /* usable phase voltage peak / Vbus */
#define VEL_LIMIT           200.0f       /* rad/s, mechanical */

/* ---- thermal ---- */
#define T_DERATE_START      100.0f       /* degC, estimated winding temperature */
#define T_DERATE_END        120.0f       /* current limit reaches 0 here */
#define T_TRIP_WINDING      125.0f
#define T_TRIP_NTC          110.0f
#define TH_RTH_W_S          2.0f         /* K/W winding -> stator (used by the estimator) */
#define TH_CTH_W            30.0f        /* J/K winding thermal mass */

/* ---- current loop ---- */
#define I_LOOP_BW_HZ        1000.0f      /* current loop bandwidth after calibration */

/* ---- calibration ---- */
#define CAL_I_ALIGN         4.0f         /* A on the d axis to hold the rotor */
#define CAL_I_LOW           2.0f
#define CAL_SWEEP_HZ        1.0f         /* electrical revolutions per second */

/* ---- joint bus (RS-485) ---- */
#define JBUS_DEFAULT_ID     1u
#define JBUS_BROADCAST      0xFEu
#define JBUS_BAUD           4000000u
#define JBUS_TURNAROUND_US  30u          /* first reply slot starts this long after a frame ends */
#define JBUS_GUARD_US       4u           /* idle time between reply slots */
#define COMM_TIMEOUT_MS     100u         /* no command for this long -> damping mode */
#define KD_SAFE             0.01f        /* Nm*s/rad, damping used after a communication loss */

#endif
