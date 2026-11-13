#ifndef HAL_STM32G4_H
#define HAL_STM32G4_H

#include "joint.h"

extern const hal_ops_t STM32G4_HAL;
void board_clock_init(void);
void board_periph_init(void);
void board_tim6_irq(void);
void board_adc_ack(void);
uint8_t board_uart_rx(uint8_t *byte);

#endif
