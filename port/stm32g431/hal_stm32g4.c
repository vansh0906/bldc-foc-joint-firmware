/* hal.h on the STM32G431, register level (no vendor HAL). Compiles and links for the target;
 * it has not been run on hardware yet: docs/BRINGUP.md lists the bench checks, in order. */
#include <string.h>

#include "board.h"
#include "hal_stm32g4.h"
#include "regs.h"

static uint8_t tx_buf[JBUS_MAX_FRAME];
static uint16_t tx_len;

#if defined(__arm__)
#define CPSID() __asm volatile("cpsid i" ::: "memory")
#define CPSIE() __asm volatile("cpsie i" ::: "memory")
#define DSB()   __asm volatile("dsb" ::: "memory")
#else   /* host syntax check of this file (tools/check_port.sh) */
#define CPSID() ((void)0)
#define CPSIE() ((void)0)
#define DSB()   ((void)0)
#endif

static void delay_cycles(uint32_t n) { for (volatile uint32_t i = 0; i < n; i++) { } }

/* ---------------- clocks ---------------- */
void board_clock_init(void)
{
    RCC_APB1ENR1 |= 1u << 28;                     /* PWREN */
    PWR_CR5 &= ~(1u << 8);                        /* range 1 boost mode (needed above 150 MHz) */
    FLASH_ACR = (FLASH_ACR & ~0xFu) | 4u | (1u << 8) | (1u << 9) | (1u << 10);  /* 4 WS, prefetch, caches */
    while ((FLASH_ACR & 0xFu) != 4u) { }
    /* PLL: HSI16 / 4 * 85 / 2 = 170 MHz */
    RCC_PLLCFGR = 2u | (3u << 4) | (85u << 8) | (1u << 24);   /* PLLSRC=HSI16, M=4, N=85, R=2, PLLREN */
    RCC_CR |= 1u << 24;                           /* PLLON */
    while (!(RCC_CR & (1u << 25))) { }
    RCC_CFGR = (RCC_CFGR & ~0xF3u) | (8u << 4) | 3u;   /* AHB /2 during the switch, SW = PLL */
    while ((RCC_CFGR & 0xCu) != 0xCu) { }
    delay_cycles(200);                            /* >= 1 us at the intermediate clock */
    RCC_CFGR &= ~0xF0u;                           /* AHB /1 */
}

/* ---------------- GPIO helpers ---------------- */
static void pin_af(uint32_t port, uint32_t pin, uint32_t af)
{
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3u << (2u * pin))) | (2u << (2u * pin));
    GPIO_OSPEEDR(port) |= 3u << (2u * pin);
    if (pin < 8u) GPIO_AFRL(port) = (GPIO_AFRL(port) & ~(0xFu << (4u * pin))) | (af << (4u * pin));
    else GPIO_AFRH(port) = (GPIO_AFRH(port) & ~(0xFu << (4u * (pin - 8u)))) | (af << (4u * (pin - 8u)));
}

static void pin_input_pullup(uint32_t port, uint32_t pin)
{
    GPIO_MODER(port) &= ~(3u << (2u * pin));
    GPIO_PUPDR(port) = (GPIO_PUPDR(port) & ~(3u << (2u * pin))) | (1u << (2u * pin));
}

/* ---------------- peripherals ---------------- */
static void pwm_init(void)
{
    RCC_APB2ENR |= 1u << 11;                      /* TIM1EN */
    pin_af(GPIOA_BASE, 8, 6); pin_af(GPIOA_BASE, 9, 6); pin_af(GPIOA_BASE, 10, 6);
    pin_af(GPIOC_BASE, 13, 4); pin_af(GPIOA_BASE, 12, 6); pin_af(GPIOB_BASE, 15, 4);
    TIM_PSC(TIM1_BASE) = 0;
    TIM_ARR(TIM1_BASE) = PWM_ARR;
    TIM_RCR(TIM1_BASE) = 1;                       /* update (load new duties) once per period */
    TIM_CCMR1(TIM1_BASE) = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);   /* PWM1, preload */
    TIM_CCMR2(TIM1_BASE) = (6u << 4) | (1u << 3) | (7u << 12);               /* CH3 PWM1, CH4 PWM2 */
    TIM_CCR4(TIM1_BASE) = PWM_ARR - 1u;           /* OC4REF edge at the counter peak = PWM center */
    TIM_CR2(TIM1_BASE) = 7u << 20;                /* TRGO2 = OC4REF -> ADC injected trigger */
    TIM_CCER(TIM1_BASE) = (1u << 0) | (1u << 2) | (1u << 4) | (1u << 6) | (1u << 8) | (1u << 10);
    TIM_BDTR(TIM1_BASE) = DEADTIME_TICKS | (1u << 11) | (1u << 10);   /* dead time, OSSR/OSSI; MOE off */
    TIM_CR1(TIM1_BASE) = (1u << 5) | (1u << 7);   /* center-aligned 1, ARPE */
    TIM_EGR(TIM1_BASE) = 1u;
    TIM_CR1(TIM1_BASE) |= 1u;                     /* counter runs always: it also paces the ADC */
}

static void encoder_halls_init(void)
{
    RCC_APB1ENR1 |= 1u << 2;                      /* TIM4EN */
    pin_af(GPIOB_BASE, 6, 2); pin_af(GPIOB_BASE, 7, 2);
    TIM_CCMR1(TIM4_BASE) = 1u | (1u << 8) | (3u << 4) | (3u << 12);   /* TI1/TI2 inputs, light filter */
    TIM_SMCR(TIM4_BASE) = 3u;                     /* encoder mode 3: count on both edges of both */
    TIM_ARR(TIM4_BASE) = 0xFFFFu;
    TIM_CR1(TIM4_BASE) = 1u;
    pin_input_pullup(HALL_PORT, HALL_SHIFT);
    pin_input_pullup(HALL_PORT, HALL_SHIFT + 1u);
    pin_input_pullup(HALL_PORT, HALL_SHIFT + 2u);
}

static void timebase_init(void)
{
    RCC_APB1ENR1 |= (1u << 0) | (1u << 4);        /* TIM2EN, TIM6EN */
    TIM_PSC(TIM2_BASE) = SYSCLK_HZ / 1000000u - 1u;   /* 1 MHz, 32-bit free running */
    TIM_ARR(TIM2_BASE) = 0xFFFFFFFFu;
    TIM_EGR(TIM2_BASE) = 1u;
    TIM_CR1(TIM2_BASE) = 1u;
    TIM_PSC(TIM6_BASE) = SYSCLK_HZ / 1000000u - 1u;   /* one-shot reply-slot timer, 1 us ticks */
    TIM_EGR(TIM6_BASE) = 1u;
    TIM_SR(TIM6_BASE) = 0;
    TIM_CR1(TIM6_BASE) = 1u << 3;                 /* OPM */
    TIM_DIER(TIM6_BASE) = 1u;
}

static void opamp_adc_init(void)
{
    RCC_APB2ENR |= 1u << 0;                       /* SYSCFG (op-amps) */
    /* PGA mode, internal output to the ADC; input pins as on the ESC1 board */
    uint32_t pga = 1u | (2u << 5) | (1u << 8) | (1u << 14);
    OPAMP1_CSR = pga;
    OPAMP2_CSR = pga;
    OPAMP3_CSR = pga;

    RCC_AHB2ENR |= 1u << 13;                      /* ADC12EN */
    ADC12_CCR = 3u << 16;                         /* HCLK / 4 = 42.5 MHz */
    uint32_t adcs[2] = { ADC1_BASE, ADC2_BASE };
    for (int k = 0; k < 2; k++) {
        uint32_t b = adcs[k];
        ADC_CR(b) &= ~(1u << 29);                 /* leave deep power-down */
        ADC_CR(b) |= 1u << 28;                    /* regulator on */
        delay_cycles(4000);                       /* >= 20 us */
        ADC_CR(b) |= 1u << 31;                    /* calibrate */
        while (ADC_CR(b) & (1u << 31)) { }
        ADC_ISR(b) = 1u;
        ADC_CR(b) |= 1u;                          /* enable */
        while (!(ADC_ISR(b) & 1u)) { }
        ADC_SMPR1(b) = 0;                         /* 2.5-cycle sampling: op-amp outputs are low impedance */
        ADC_SMPR2(b) = 0;
    }
    /* injected sequences, triggered on the rising edge of TIM1 TRGO2 (JEXTSEL 8) */
    ADC_JSQR(ADC1_BASE) = 1u | (8u << 2) | (1u << 7) | (ADC1_JCH_IA << 9) | (ADC1_JCH_VBUS << 15);
    ADC_JSQR(ADC2_BASE) = 1u | (8u << 2) | (1u << 7) | (ADC2_JCH_IB << 9) | (ADC2_JCH_IC << 15);
    ADC_IER(ADC1_BASE) = 1u << 6;                 /* JEOS interrupt -> current loop */
    ADC_CR(ADC1_BASE) |= 1u << 3;                 /* JADSTART: wait for triggers */
    ADC_CR(ADC2_BASE) |= 1u << 3;
}

static void uart_init(void)
{
    RCC_APB1ENR1 |= 1u << 17;                     /* USART2EN */
    RCC_AHB1ENR |= (1u << 0) | (1u << 2);         /* DMA1, DMAMUX1 */
    pin_af(GPIOB_BASE, 3, 7); pin_af(GPIOB_BASE, 4, 7); pin_af(GPIOA_BASE, 1, 7);
    USART_BRR(USART2_BASE) = 0x52u;               /* 4 Mbaud with OVER8: div 85 */
    USART_CR3(USART2_BASE) = (1u << 14) | (1u << 7);   /* driver-enable mode, DMA transmit */
    USART_CR1(USART2_BASE) = (1u << 15) | (2u << 21) | (2u << 16) | (1u << 5) | (1u << 3) | (1u << 2) | 1u;
    DMAMUX1_C0CR = DMAMUX_REQ_USART2_TX;
    DMA1_CPAR1 = (uint32_t)(uintptr_t)&USART_TDR(USART2_BASE);
}

static void wdg_init(void)
{
    IWDG_KR = 0x5555u;
    IWDG_PR = 4u;                                 /* LSI 32 kHz / 64 */
    IWDG_RLR = 25u;                               /* 50 ms */
    while (IWDG_SR) { }
    IWDG_KR = 0xCCCCu;
}

static void nvic(uint32_t irq, uint8_t prio)
{
    NVIC_IPR8(irq) = prio;
    NVIC_ISER(irq >> 5) = 1u << (irq & 31u);
}

void board_periph_init(void)
{
    RCC_AHB2ENR |= 7u;                            /* GPIOA..C */
    timebase_init();
    pwm_init();
    encoder_halls_init();
    opamp_adc_init();
    uart_init();
    wdg_init();
    nvic(IRQ_ADC1_2, 0x00);                       /* above FreeRTOS: never masked, no RTOS calls */
    nvic(IRQ_USART2, 0x60);                       /* below configMAX_SYSCALL (0x50): may notify */
    nvic(IRQ_TIM6_DAC, 0x60);
}

/* ---------------- hal_ops ---------------- */
static void h_read_adc(void *c, uint16_t i[3], uint16_t *vbus)
{
    (void)c;
    i[0] = (uint16_t)ADC_JDR1(ADC1_BASE);
    i[1] = (uint16_t)ADC_JDR1(ADC2_BASE);
    i[2] = (uint16_t)ADC_JDR2(ADC2_BASE);
    *vbus = (uint16_t)ADC_JDR2(ADC1_BASE);
}

static uint16_t h_read_ntc(void *c)
{
    (void)c;
    REG(ADC1_BASE + 0x30u) = (uint32_t)ADC1_CH_NTC << 6;   /* SQR1: one conversion */
    ADC_CR(ADC1_BASE) |= 1u << 2;                 /* ADSTART */
    for (uint32_t n = 0; n < 100000u && !(ADC_ISR(ADC1_BASE) & (1u << 2)); n++) { }
    return (uint16_t)REG(ADC1_BASE + 0x40u);      /* DR */
}

static uint16_t h_read_encoder(void *c) { (void)c; return (uint16_t)TIM_CNT(TIM4_BASE); }
static uint8_t h_read_halls(void *c) { (void)c; return (uint8_t)((GPIO_IDR(HALL_PORT) >> HALL_SHIFT) & 7u); }

static void h_set_duty(void *c, const float d[3])
{
    (void)c;
    TIM_CCR1(TIM1_BASE) = (uint32_t)(d[0] * (float)PWM_ARR);
    TIM_CCR2(TIM1_BASE) = (uint32_t)(d[1] * (float)PWM_ARR);
    TIM_CCR3(TIM1_BASE) = (uint32_t)(d[2] * (float)PWM_ARR);
}

static void h_pwm_enable(void *c, bool on)
{
    (void)c;
    if (on) {
        TIM_EGR(TIM1_BASE) = 1u;                  /* load the duties written just before */
        TIM_BDTR(TIM1_BASE) |= 1u << 15;          /* MOE */
    } else {
        TIM_BDTR(TIM1_BASE) &= ~(1u << 15);
    }
}

static uint32_t h_micros(void *c) { (void)c; return TIM_CNT(TIM2_BASE); }

static void dma_start(void)
{
    DMA1_CCR1 = 0;
    DMA1_IFCR = 0xFu;
    DMA1_CMAR1 = (uint32_t)(uintptr_t)tx_buf;
    DMA1_CNDTR1 = tx_len;
    DMA1_CCR1 = (1u << 7) | (1u << 4) | 1u;       /* MINC, memory -> peripheral, enable */
}

static void h_uart_send(void *c, const uint8_t *buf, uint16_t n, uint32_t t_start)
{
    (void)c;
    if (n > sizeof tx_buf) return;
    memcpy(tx_buf, buf, n);
    tx_len = n;
    int32_t wait = (int32_t)(t_start - TIM_CNT(TIM2_BASE));
    if (wait <= 1) { dma_start(); return; }
    TIM_ARR(TIM6_BASE) = (uint32_t)wait;          /* the reply slot starts exactly on time */
    TIM_CNT(TIM6_BASE) = 0;
    TIM_CR1(TIM6_BASE) |= 1u;
}

void board_tim6_irq(void)
{
    TIM_SR(TIM6_BASE) = 0;
    dma_start();
}

static void h_wdg_kick(void *c) { (void)c; IWDG_KR = 0xAAAAu; }

static bool flash_wait(void)
{
    for (uint32_t n = 0; n < 4000000u; n++)
        if (!(FLASH_SR & (1u << 16))) return (FLASH_SR & 0xFAu) == 0u;   /* no error flags */
    return false;
}

static bool h_flash_write(void *c, const void *data, uint16_t n)
{
    (void)c;
    /* called only with the drive off (state IDLE/FAULT): erasing stalls the CPU for ~20 ms */
    bool ok = true;
    FLASH_KEYR = 0x45670123u;
    FLASH_KEYR = 0xCDEF89ABu;
    FLASH_SR = 0xFFFFu;
    FLASH_CR = (1u << 1) | (PARAMS_FLASH_PAGE << 3);
    FLASH_CR |= 1u << 16;                         /* STRT: erase the page */
    ok = flash_wait();
    FLASH_CR = 1u;                                /* PG */
    const uint8_t *src = data;
    for (uint16_t off = 0; ok && off < n; off = (uint16_t)(off + 8u)) {
        uint32_t w[2] = { 0xFFFFFFFFu, 0xFFFFFFFFu };
        uint16_t rem = (uint16_t)(n - off);
        memcpy(w, src + off, rem < 8u ? (size_t)rem : (size_t)8u);
        REG(PARAMS_FLASH_ADDR + off) = w[0];
        REG(PARAMS_FLASH_ADDR + off + 4u) = w[1];
        ok = flash_wait();
    }
    FLASH_CR = 1u << 31;                          /* lock */
    return ok;
}

static bool h_flash_read(void *c, void *data, uint16_t n)
{
    (void)c;
    memcpy(data, (const void *)(uintptr_t)PARAMS_FLASH_ADDR, n);
    return true;
}

static void h_irq_disable(void *c) { (void)c; CPSID(); }
static void h_irq_enable(void *c) { (void)c; DSB(); CPSIE(); }

const hal_ops_t STM32G4_HAL = {
    h_read_adc, h_read_ntc, h_read_encoder, h_read_halls, h_set_duty, h_pwm_enable,
    h_uart_send, h_micros, h_wdg_kick, h_flash_write, h_flash_read, h_irq_disable, h_irq_enable
};

/* USART2 RX: one byte at a time into the joint's ring buffer */
uint8_t board_uart_rx(uint8_t *byte)
{
    uint32_t isr = USART_ISR(USART2_BASE);
    if (isr & (1u << 3)) USART_ICR(USART2_BASE) = 1u << 3;   /* overrun: clear, the CRC will catch it */
    if (!(isr & (1u << 5))) return 0;
    *byte = (uint8_t)USART_RDR(USART2_BASE);
    return 1;
}

void board_adc_ack(void) { ADC_ISR(ADC1_BASE) = 1u << 6; }
