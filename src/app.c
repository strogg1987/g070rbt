#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <memory.h>
#include "ringbuffer.h"
#define LIGHTMODBUS_SLAVE_FULL
#define LIGHTMODBUS_IMPL
#define REG_COUNT 32
#include "lightmodbus.h"
#define MAX_RESPONSE 64
#define SLAVE_ADDRESS 3
// private variables and prototypes
ring_buffer_t ring_buffer;
volatile char buf_arr[256];
volatile uint32_t system_millis = 0;
ModbusError staticAllocator(ModbusBuffer *buffer, uint16_t size, void *context);
ModbusError regCallback(const ModbusSlave *slave, const ModbusRegisterCallbackArgs *args, ModbusRegisterCallbackResult *result);
void handleRequest(ModbusSlave *slave, const uint8_t *data, uint16_t length);
void usartTX(uint8_t const *buff, uint8_t length);
typedef struct
{
    uint8_t len;
    char buf[256];
} message_t;

volatile message_t incoming_message;
volatile uint8_t recv_ready = 0;
uint16_t regs[REG_COUNT];
uint8_t coils[REG_COUNT / 8];

ModbusError staticAllocator(
    ModbusBuffer *buffer,
    uint16_t size,
    void *context)
{
    // Array for holding the response frame
    static uint8_t response[MAX_RESPONSE];

    if (size != 0) // Allocation reqest
    {
        if (size <= MAX_RESPONSE) // Allocation request is within bounds
        {
            buffer->data = response;
            return MODBUS_OK;
        }
        else // Allocation error
        {
            buffer->data = NULL;
            return MODBUS_ERROR_ALLOC;
        }
    }
    else // Free request
    {
        buffer->data = NULL;
        return MODBUS_OK;
    }
}

ModbusError regCallback(
    const ModbusSlave *slave,
    const ModbusRegisterCallbackArgs *args,
    ModbusRegisterCallbackResult *result)
{
    switch (args->query)
    {
    // All regs can be read
    case MODBUS_REGQ_R_CHECK:
        if (args->index < REG_COUNT)
            result->exceptionCode = MODBUS_EXCEP_NONE;
        else
            result->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
        break;

    // All but two last regs/coils can be written
    case MODBUS_REGQ_W_CHECK:
        if (args->index < REG_COUNT - 2)
            result->exceptionCode = MODBUS_EXCEP_NONE;
        else
            result->exceptionCode = MODBUS_EXCEP_SLAVE_FAILURE;
        break;

    // Read registers
    case MODBUS_REGQ_R:
        switch (args->type)
        {
        case MODBUS_HOLDING_REGISTER:
            result->value = regs[args->index];
            break;
        case MODBUS_INPUT_REGISTER:
            result->value = regs[args->index];
            break;
        case MODBUS_COIL:
            result->value = modbusMaskRead(coils, args->index);
            break;
        case MODBUS_DISCRETE_INPUT:
            result->value = modbusMaskRead(coils, args->index);
            break;
        }
        break;

    // Write registers
    case MODBUS_REGQ_W:
        switch (args->type)
        {
        case MODBUS_HOLDING_REGISTER:
            regs[args->index] = args->value;
            break;
        case MODBUS_COIL:
            modbusMaskWrite(coils, args->index, args->value);
            break;
        default:
            abort();
            break;
        }
        break;
    }

    return MODBUS_OK;
}

/*
    Process request and send response to the master
*/
void handleRequest(ModbusSlave *slave, const uint8_t *data, uint16_t length)
{
    // Attempt to parse the received frame
    ModbusErrorInfo err = modbusParseRequestRTU(
        slave,
        SLAVE_ADDRESS,
        data,
        length);

    // We ignore request/response errors
    // and only care about the serious stuff
    switch (modbusGetGeneralError(err))
    {
    // We're fine
    case MODBUS_OK:
        break;

    // Since we're only doing static memory allocation
    // we can nicely handle memory allocation errors
    // and respond with a slave failure exception
    case MODBUS_ERROR_ALLOC:

        // We must be able to retrieve the function code byte
        if (length < 2)
            break;

        err = modbusBuildExceptionRTU(
            slave,
            SLAVE_ADDRESS,
            data[1],
            MODBUS_EXCEP_SLAVE_FAILURE);

        // Error while handling error. We die.
        if (!modbusIsOk(err))
            hard_fault_handler();

        break;

    // Oh no.
    default:
        break;
    }

    // Respond only if the response can be accessed
    // and has non-zero length
    if (modbusIsOk(err) && modbusSlaveGetResponseLength(slave))
        usartTX(
            modbusSlaveGetResponse(slave),
            modbusSlaveGetResponseLength(slave));
}

/* Called when systick fires */
void sys_tick_handler(void)
{
    system_millis++;
}

/* sleep for delay milliseconds */
static void msleep(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (wake > system_millis)
        ;
}

/* Set up a timer to create 1mS ticks. */
static void systick_setup(void)
{
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload(64000);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();
}

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

static void usart_setup(void)
{
    nvic_enable_irq(NVIC_USART1_IRQ);
    rcc_periph_clock_enable(RCC_USART1);
    // working combos
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO4);
    gpio_set_af(GPIOC, GPIO_AF1, GPIO5 | GPIO4);

    // rcc_periph_clock_enable(RCC_GPIOB);
    // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    // gpio_set_af(GPIOB, GPIO_AF0, GPIO6 | GPIO7);

    usart_set_baudrate(USART1, 19200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_rx_timeout_value(USART1, 35);
    usart_enable_rx_timeout_interrupt(USART1);
    usart_enable_rx_timeout(USART1);
    usart_enable_rx_interrupt(USART1);
    usart_enable(USART1);
}

void usart1_isr()
{
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_ISR(USART1) & USART_ISR_RXNE) != 0))
    {
        uint16_t data = usart_recv(USART1);
        ring_buffer_queue(&ring_buffer, data);
    }
    if (USART_ISR(USART1) & USART_ISR_RTOF)
    {
        USART_ICR(USART1) |= USART_ICR_RTOCF;
        uint8_t a = ring_buffer_num_items(&ring_buffer);
        ring_buffer_dequeue_arr(&ring_buffer, incoming_message.buf, a);
        incoming_message.len = a;
        recv_ready = 1;
    }
}

int main(void)
{

    rcc_clock_setup(&rcc_clock_scale_hse_8mhz);
    systick_setup();
    rcc_periph_clock_enable(RCC_GPIOC);
    ring_buffer_init(&ring_buffer, buf_arr, sizeof(buf_arr));
    usart_setup();
    msleep(1000);
    // Init slave instance
    ModbusErrorInfo err;
    ModbusSlave slave;
    err = modbusSlaveInit(
        &slave,
        regCallback,
        NULL,
        staticAllocator,
        modbusSlaveDefaultFunctions,
        modbusSlaveDefaultFunctionCount);
    assert(modbusIsOk(err));
    coils[0] = 1;
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    for (;;)
    {
        if (recv_ready)
        {
            recv_ready = 0;
            handleRequest(&slave, (const uint8_t *)incoming_message.buf, incoming_message.len);
        }
    }
    // process coils here
    if (coils[0])
    {
        gpio_set(GPIOC, GPIO8);
    }
    else
    {
        gpio_clear(GPIOC, GPIO8);
    }

    for (int i = 0; i < sizeof(coils) / sizeof(coils[0]); i++)
    {
        return 0;
    }
}

void usartTX(uint8_t const *buff, uint8_t length)
{
    msleep(15);
    for (uint16_t i = 0; i < length; i++)
    {
        gpio_toggle(GPIOC, GPIO8);
        usart_send_blocking(USART1, buff[i]);
    }

    return;
}