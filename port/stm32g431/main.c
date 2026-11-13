/* Target entry: FreeRTOS tasks around the joint controller.
 *
 *   ADC injected end-of-sequence IRQ, 20 kHz, priority 0  -> app_isr (current loop)
 *       above configMAX_SYSCALL_INTERRUPT_PRIORITY: never masked by the kernel, no RTOS calls
 *   ctrl task   1 kHz  (vTaskDelayUntil)   -> app_ctrl_tick (outer loops, protections)
 *   comm task   wakes on every received byte (task notification) or 1 ms -> app_comm_poll
 *   monitor     100 Hz -> app_monitor (temperature, watchdog kick if ctrl+comm are alive) */
#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "hal_stm32g4.h"

static app_t g_app;
static TaskHandle_t h_comm;

static StaticTask_t tcb_ctrl, tcb_comm, tcb_mon;
static StackType_t stk_ctrl[384], stk_comm[384], stk_mon[256];

void ADC1_2_IRQHandler(void);
void USART2_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);

void ADC1_2_IRQHandler(void)
{
    board_adc_ack();
    app_isr(&g_app);
}

void USART2_IRQHandler(void)
{
    uint8_t b;
    BaseType_t woken = pdFALSE;
    while (board_uart_rx(&b)) app_uart_rx(&g_app, b, g_app.hal.ops->micros(g_app.hal.ctx));
    vTaskNotifyGiveFromISR(h_comm, &woken);
    portYIELD_FROM_ISR(woken);
}

void TIM6_DAC_IRQHandler(void) { board_tim6_irq(); }

static void ctrl_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, 1);
        app_ctrl_tick(&g_app);
    }
}

static void comm_task(void *arg)
{
    (void)arg;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, 1);
        app_comm_poll(&g_app);
    }
}

static void mon_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, 10);
        app_monitor(&g_app);
    }
}

int main(void)
{
    board_clock_init();
    hal_t hal = { &STM32G4_HAL, 0 };
    app_init(&g_app, hal);              /* PWM outputs stay off until ENABLE */
    board_periph_init();                /* starts the PWM timer -> ADC -> app_isr (BOOT: offsets) */
    xTaskCreateStatic(ctrl_task, "ctrl", 384, 0, 3, stk_ctrl, &tcb_ctrl);
    h_comm = xTaskCreateStatic(comm_task, "comm", 384, 0, 2, stk_comm, &tcb_comm);
    xTaskCreateStatic(mon_task, "mon", 256, 0, 1, stk_mon, &tcb_mon);
    vTaskStartScheduler();
    for (;;) { }
}

/* static allocation for the kernel's own tasks */
static StaticTask_t idle_tcb;
static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, configSTACK_DEPTH_TYPE *size)
{
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *size = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    (void)t; (void)name;
    g_app.hal.ops->pwm_enable(g_app.hal.ctx, false);
    for (;;) { }                        /* the watchdog resets the chip */
}
