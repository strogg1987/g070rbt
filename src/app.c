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
// private variables and prototypes
volatile uint32_t sys_millis;
void config_timer16(void);
void config_timer17(void);
uint8_t channel_array[] = {1, 1, ADC_CHANNEL_TEMP};
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
    volatile uint8_t local = 0;
    for (;;)
    {
        adc_start_conversion_regular(ADC1);
        while (!(adc_eoc(ADC1)))
            ;
        volatile uint16_t temp = adc_read_regular(ADC1);
        gpio_toggle(GPIOC, GPIO8);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void usart_setup(void)
{
    /* Setup USART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);
}

void config_timer16(void)
{
    rcc_periph_clock_enable(RCC_TIM16);
    timer_set_prescaler(TIM16, 7);
    timer_enable_preload(TIM16);
    timer_set_period(TIM16, 1000);
    timer_set_oc_value(TIM16, TIM_OC1, 250);
    timer_set_counter(TIM16, 0);
    timer_enable_oc_preload(TIM16, TIM_OC1);
    timer_set_oc_mode(TIM16, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM16, TIM_OC1);
    timer_continuous_mode(TIM16);
    timer_generate_event(TIM16, TIM_EGR_UG);
    timer_enable_counter(TIM16);
    // timer_enable_irq(TIM16, TIM_DIER_COMIE);  //enable commutation interrupt
    // nvic_enable_irq(NVIC_TIM1_CC_IRQ);
}

void config_timer17(void)
{
    rcc_periph_clock_enable(RCC_TIM17); // enable
    timer_set_prescaler(TIM17, 7);
    timer_enable_preload(TIM17);
    timer_set_period(TIM17, 1000);
    timer_set_oc_value(TIM17, TIM_OC1, 250);
    timer_set_counter(TIM17, 0);
    timer_enable_oc_preload(TIM17, TIM_OC1);
    timer_set_oc_mode(TIM17, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM17, TIM_OC1);
    timer_continuous_mode(TIM17);
    timer_generate_event(TIM17, TIM_EGR_UG);
    timer_enable_counter(TIM17);
}

static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);
    adc_power_off(ADC1);
    adc_set_clk_prescale(ADC1, ADC_CCR_PRESC_DIV2);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_160DOT5);
    uint8_t channel = 0;
    adc_set_regular_sequence(ADC1, 1, &channel);
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_power_on(ADC1);
    adc_start_conversion_regular(ADC1);
}

int main(void)
{

    rcc_clock_setup(&rcc_clock_scale_hse_8mhz);
    rcc_periph_clock_enable(RCC_GPIOC);
    config_timer16();
    config_timer17();
    adc_setup();
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    // Tell FreeRTOS about our toggle task, and set it's stack and priority
    xTaskCreate(task1, "LED", 100, NULL, 2, NULL);
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}