/* Board wiring. Power stage and current sensing follow ST's B-G431B-ESC1 (three low-side
 * shunts into the G4's internal op-amps); the RS-485 transceiver, hall and encoder inputs
 * are on a carrier board. CHECK EVERY LINE against your schematic before powering up
 * (docs/BRINGUP.md). */
#ifndef BOARD_H
#define BOARD_H

#define SYSCLK_HZ        170000000u
#define PWM_ARR          (SYSCLK_HZ / 20000u / 2u)    /* center-aligned: 4250 counts per half period */
#define DEADTIME_TICKS   34u                          /* 200 ns at 170 MHz */

/* TIM1 PWM: PA8/PA9/PA10 = CH1..3 (AF6), PC13/PA12/PB15 = CH1N..3N (AF4/AF6/AF4) */
/* Encoder A/B: PB6/PB7 = TIM4 CH1/CH2 (AF2) */
/* Halls A/B/C: PC6, PC7, PC8 inputs with pull-ups */
#define HALL_PORT        GPIOC_BASE
#define HALL_SHIFT       6u
/* RS-485: USART2 TX PB3, RX PB4 (AF7), DE PA1 (AF7, driver enable driven by the USART) */

/* ADC injected channels (sampled at the PWM center, triggered by TIM1 TRGO2) */
#define ADC1_JCH_IA      13u     /* OPAMP1 output, internal */
#define ADC1_JCH_VBUS    1u      /* PA0, 1:19 divider */
#define ADC2_JCH_IB      16u     /* OPAMP2 output, internal */
#define ADC2_JCH_IC      18u     /* OPAMP3 output, internal */
#define ADC1_CH_NTC      5u      /* PB14 via a regular conversion, read by the monitor task */

/* last 2 KB page of the 128 KB flash keeps the parameters */
#define PARAMS_FLASH_ADDR 0x0801F800u
#define PARAMS_FLASH_PAGE 63u

#endif
