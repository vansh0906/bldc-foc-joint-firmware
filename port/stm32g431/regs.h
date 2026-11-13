/* Minimal STM32G431 register map (RM0440). Only what this port touches; no vendor headers. */
#ifndef REGS_H
#define REGS_H

#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* RCC */
#define RCC_BASE        0x40021000u
#define RCC_CR          REG(RCC_BASE + 0x00u)
#define RCC_CFGR        REG(RCC_BASE + 0x08u)
#define RCC_PLLCFGR     REG(RCC_BASE + 0x0Cu)
#define RCC_AHB1ENR     REG(RCC_BASE + 0x48u)
#define RCC_AHB2ENR     REG(RCC_BASE + 0x4Cu)
#define RCC_APB1ENR1    REG(RCC_BASE + 0x58u)
#define RCC_APB2ENR     REG(RCC_BASE + 0x60u)
#define RCC_CCIPR       REG(RCC_BASE + 0x88u)
#define RCC_CSR         REG(RCC_BASE + 0x94u)

/* FLASH */
#define FLASH_BASE_R    0x40022000u
#define FLASH_ACR       REG(FLASH_BASE_R + 0x00u)
#define FLASH_KEYR      REG(FLASH_BASE_R + 0x08u)
#define FLASH_SR        REG(FLASH_BASE_R + 0x10u)
#define FLASH_CR        REG(FLASH_BASE_R + 0x14u)

/* PWR */
#define PWR_CR5         REG(0x40007000u + 0x80u)

/* GPIO */
#define GPIOA_BASE      0x48000000u
#define GPIOB_BASE      0x48000400u
#define GPIOC_BASE      0x48000800u
#define GPIO_MODER(b)   REG((b) + 0x00u)
#define GPIO_OSPEEDR(b) REG((b) + 0x08u)
#define GPIO_PUPDR(b)   REG((b) + 0x0Cu)
#define GPIO_IDR(b)     REG((b) + 0x10u)
#define GPIO_AFRL(b)    REG((b) + 0x20u)
#define GPIO_AFRH(b)    REG((b) + 0x24u)

/* general timer layout (TIM1/2/4/6) */
#define TIM_CR1(b)      REG((b) + 0x00u)
#define TIM_CR2(b)      REG((b) + 0x04u)
#define TIM_SMCR(b)     REG((b) + 0x08u)
#define TIM_DIER(b)     REG((b) + 0x0Cu)
#define TIM_SR(b)       REG((b) + 0x10u)
#define TIM_EGR(b)      REG((b) + 0x14u)
#define TIM_CCMR1(b)    REG((b) + 0x18u)
#define TIM_CCMR2(b)    REG((b) + 0x1Cu)
#define TIM_CCER(b)     REG((b) + 0x20u)
#define TIM_CNT(b)      REG((b) + 0x24u)
#define TIM_PSC(b)      REG((b) + 0x28u)
#define TIM_ARR(b)      REG((b) + 0x2Cu)
#define TIM_RCR(b)      REG((b) + 0x30u)
#define TIM_CCR1(b)     REG((b) + 0x34u)
#define TIM_CCR2(b)     REG((b) + 0x38u)
#define TIM_CCR3(b)     REG((b) + 0x3Cu)
#define TIM_CCR4(b)     REG((b) + 0x40u)
#define TIM_BDTR(b)     REG((b) + 0x44u)
#define TIM1_BASE       0x40012C00u
#define TIM2_BASE       0x40000000u
#define TIM4_BASE       0x40000800u
#define TIM6_BASE       0x40001000u

/* ADC */
#define ADC1_BASE       0x50000000u
#define ADC2_BASE       0x50000100u
#define ADC_ISR(b)      REG((b) + 0x00u)
#define ADC_IER(b)      REG((b) + 0x04u)
#define ADC_CR(b)       REG((b) + 0x08u)
#define ADC_CFGR(b)     REG((b) + 0x0Cu)
#define ADC_SMPR1(b)    REG((b) + 0x14u)
#define ADC_SMPR2(b)    REG((b) + 0x18u)
#define ADC_JSQR(b)     REG((b) + 0x4Cu)
#define ADC_JDR1(b)     REG((b) + 0x80u)
#define ADC_JDR2(b)     REG((b) + 0x84u)
#define ADC12_CCR       REG(0x50000300u + 0x08u)

/* OPAMP (current-sense amplifiers in PGA mode) */
#define OPAMP1_CSR      REG(0x40010300u + 0x00u)
#define OPAMP2_CSR      REG(0x40010300u + 0x04u)
#define OPAMP3_CSR      REG(0x40010300u + 0x08u)

/* USART2 */
#define USART2_BASE     0x40004400u
#define USART_CR1(b)    REG((b) + 0x00u)
#define USART_CR3(b)    REG((b) + 0x08u)
#define USART_BRR(b)    REG((b) + 0x0Cu)
#define USART_ISR(b)    REG((b) + 0x1Cu)
#define USART_ICR(b)    REG((b) + 0x20u)
#define USART_RDR(b)    REG((b) + 0x24u)
#define USART_TDR(b)    REG((b) + 0x28u)

/* DMA1 channel 1 + DMAMUX for USART2 TX */
#define DMA1_IFCR       REG(0x40020000u + 0x04u)
#define DMA1_CCR1       REG(0x40020000u + 0x08u)
#define DMA1_CNDTR1     REG(0x40020000u + 0x0Cu)
#define DMA1_CPAR1      REG(0x40020000u + 0x10u)
#define DMA1_CMAR1      REG(0x40020000u + 0x14u)
#define DMAMUX1_C0CR    REG(0x40020800u + 0x00u)
#define DMAMUX_REQ_USART2_TX 27u

/* IWDG */
#define IWDG_KR         REG(0x40003000u + 0x00u)
#define IWDG_PR         REG(0x40003000u + 0x04u)
#define IWDG_RLR        REG(0x40003000u + 0x08u)
#define IWDG_SR         REG(0x40003000u + 0x0Cu)

/* core */
#define SCB_CPACR       REG(0xE000ED88u)
#define SCB_VTOR        REG(0xE000ED08u)
#define NVIC_ISER(n)    REG(0xE000E100u + 4u * (n))
#define NVIC_IPR8(n)    (*(volatile uint8_t *)(uintptr_t)(0xE000E400u + (n)))

/* interrupt numbers */
#define IRQ_ADC1_2      18u
#define IRQ_USART2      38u
#define IRQ_TIM6_DAC    54u

#endif
