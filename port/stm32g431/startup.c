/* Vector table and reset handler for the STM32G431 (Cortex-M4F). */
#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
int main(void);
void Reset_Handler(void);
void Default_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void ADC1_2_IRQHandler(void);
void USART2_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);

void Default_Handler(void)
{
    /* unexpected interrupt or fault: switch the bridge off and wait for the watchdog */
    *(volatile uint32_t *)0x40012C44u &= ~(1u << 15);    /* TIM1_BDTR.MOE = 0 */
    for (;;) { }
}

void Reset_Handler(void)
{
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss;) *dst++ = 0;
    *(volatile uint32_t *)0xE000ED88u |= 0xFu << 20;     /* FPU full access */
    __asm volatile("dsb\n isb");
    main();
    for (;;) { }
}

#define NVEC (16 + 102)
typedef void (*vec_t)(void);

__attribute__((section(".isr_vector"), used))
const vec_t vector_table[NVEC] = {
    [0] = (vec_t)(uintptr_t)&_estack,
    [1] = Reset_Handler,
    [2] = Default_Handler, [3] = Default_Handler, [4] = Default_Handler, [5] = Default_Handler,
    [6] = Default_Handler, [11] = SVC_Handler, [12] = Default_Handler, [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 + 18] = ADC1_2_IRQHandler,
    [16 + 38] = USART2_IRQHandler,
    [16 + 54] = TIM6_DAC_IRQHandler,
};
