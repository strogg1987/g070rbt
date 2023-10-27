#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
// private variables and prototypes
volatile uint32_t sys_millis;
void config_timer16(void);
void config_timer17(void);

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

void sys_tick_handler(void)
{
    sys_millis++;
}

static void init_systick(void)
{
    systick_set_reload(64000);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

static void delay_ms(uint32_t ms)
{
    uint32_t wake = sys_millis + ms;
    while (wake > sys_millis)
        ;
}

void config_timer16(void)
{
    rcc_periph_clock_enable(RCC_TIM16);
    timer_set_prescaler(TIM16, 7);           // s/b 8MHz
    timer_enable_preload(TIM16);             // causes counter to be loaded from its ARR only at next update event
    timer_set_period(TIM16, 1000);           // set the timer period in the (ARR) auto-reload register
    timer_set_oc_value(TIM16, TIM_OC1, 250); // set duty cycle to 25%
    timer_set_counter(TIM16, 0);             // TIM_CNT
    timer_enable_oc_preload(TIM16, TIM_OC1);
    timer_set_oc_mode(TIM16, TIM_OC1, TIM_OCM_PWM1); // output active when counter is lt compare register
    timer_enable_oc_output(TIM16, TIM_OC1);          // enable timer output compare
    timer_continuous_mode(TIM16);                    // enable the timer to run continuously
    timer_generate_event(TIM16, TIM_EGR_UG);         // required!
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

int main(void)
{

    rcc_clock_setup(&rcc_clock_scale_hse_8mhz);
    init_systick();
    rcc_periph_clock_enable(RCC_GPIOC);
    config_timer16();
    config_timer17();
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    for (;;)
    {
        gpio_toggle(GPIOC, GPIO8);
        delay_ms(100);
    }
}