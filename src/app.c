#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include "FreeRTOS.h"
#include "task.h"
#include <memory.h>
#include "nanomodbus.h"

#define COILS_ADDR_MAX 16
#define REGS_ADDR_MAX 32
#define RTU_SERVER_ADDRESS 1
// private variables and prototypes
volatile uint32_t sys_millis;
uint8_t channel_array[] = {1, 1, ADC_CHANNEL_TEMP};
typedef struct
{
    char data[50];
    uint8_t len;
} buffer;

buffer msg;
buffer rx_buffer;

const struct rcc_clock_scale rcc_clock_scale_hse_8mhz = {
    /* 64mhz from hse@8mhz via pll @ 128mhz / 2, scale1, 2ws */
    .sysclock_source = RCC_PLL,
    .pll_source = RCC_PLLCFGR_PLLSRC_HSE,
    .pll_div = RCC_PLLCFGR_PLLM_DIV(2),
    .pll_mul = RCC_PLLCFGR_PLLN_MUL(32),
    .pllp_div = RCC_PLLCFGR_PLLP_DIV(2),
    .pllq_div = RCC_PLLCFGR_PLLQ_DIV(2),
    .pllr_div = RCC_PLLCFGR_PLLR_DIV(2),
    .hpre = RCC_CFGR_HPRE_NODIV,
    .ppre = RCC_CFGR_PPRE_NODIV,
    .flash_waitstates = FLASH_ACR_LATENCY_2WS,
    .voltage_scale = PWR_SCALE1,
    .ahb_frequency = 64000000,
    .apb_frequency = 64000000,
};
void vApplicationStackOverflowHook(
    TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused)))
{

    for (;;)
        ;
}

static void task1(void *args __attribute__((unused)))
{
    for (;;)
    {
        usart_send_blocking(USART1, '0');
        gpio_toggle(GPIOC, GPIO8);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void usart_setup(void)
{
    nvic_enable_irq(NVIC_USART1_IRQ);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOC);
    // gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5|GPIO4);
    // gpio_set_af(GPIOC, GPIO_AF0, GPIO5|GPIO4);

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF0, GPIO6 | GPIO7);

    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_rx_timeout_value(USART1, 22);
    usart_enable_rx_timeout_interrupt(USART1);

    usart_enable_rx_interrupt(USART1);
    usart_enable(USART1);
}

void usart1_isr()
{
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_ISR(USART1) & USART_ISR_RXNE) != 0))
    {
        rx_buffer.data[rx_buffer.len] = usart_recv(USART1);
        rx_buffer.len++;
    }
    if ((USART_CR1(USART1) & USART_CR1_RTOIE) != 0)
    {
        usart_disable_rx_interrupt(USART1);
        memcpy(msg.data, rx_buffer.data, rx_buffer.len);
        msg.len = rx_buffer.len;
        rx_buffer.len = 0;
    }
}

int main(void)
{

    rcc_clock_setup(&rcc_clock_scale_hse_8mhz);
    rcc_periph_clock_enable(RCC_GPIOC);
    usart_setup();
    rx_buffer.len = 0;
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    xTaskCreate(task1, "LED", 100, NULL, 2, NULL);
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}