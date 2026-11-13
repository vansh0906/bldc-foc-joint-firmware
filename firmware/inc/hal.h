/* Hardware abstraction. The control code only talks to hardware through these calls, so the
 * same code runs on the STM32G431 (port/stm32g431) and against the motor model (sim/hal_sim.c). */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* sampled at the PWM center (low-side switches on), read by the current-loop ISR */
    void (*read_adc)(void *ctx, uint16_t i_raw[3], uint16_t *vbus_raw);
    uint16_t (*read_ntc)(void *ctx);                 /* slow channel, read by the monitor */
    uint16_t (*read_encoder)(void *ctx);             /* 16-bit quadrature timer count */
    uint8_t (*read_halls)(void *ctx);                /* bit0 = A, bit1 = B, bit2 = C */
    void (*set_duty)(void *ctx, const float duty[3]); /* takes effect at the next PWM period */
    void (*pwm_enable)(void *ctx, bool on);          /* off = all six switches off (coast) */

    /* RS-485: send n bytes starting at absolute bus time t_start_us (driver enable is handled
     * by the UART hardware); a timer starts the DMA so reply slots are exact */
    void (*uart_send)(void *ctx, const uint8_t *buf, uint16_t n, uint32_t t_start_us);
    uint32_t (*micros)(void *ctx);

    void (*wdg_kick)(void *ctx);
    bool (*flash_write)(void *ctx, const void *data, uint16_t n);
    bool (*flash_read)(void *ctx, void *data, uint16_t n);

    /* interrupt masking around multi-word updates shared between ISR and tasks */
    void (*irq_disable)(void *ctx);
    void (*irq_enable)(void *ctx);
} hal_ops_t;

typedef struct {
    const hal_ops_t *ops;
    void *ctx;
} hal_t;

#endif
