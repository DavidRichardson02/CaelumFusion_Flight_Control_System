// CaelumFusion EK-TM4C123GXL -> Basys-3 fixed-packet UART producer.
//
// Confirmed LaunchPad-side contract from the EK-TM4C123GXL user manual:
//   PC5 / U1TX / J4.05 -> Basys-3 JXADC pin 1 / teensy_uart_rx_raw
//   PC4 / U1RX / J4.04 <- Basys-3 JXADC pin 2 / teensy_uart_tx (optional)
//   GND / J2.01 or J3.02 -> Basys-3 JXADC pin 5 or 11 GND
//
// Do not connect LaunchPad J3.01 5.0 V to the Basys-3 or any FPGA I/O. For
// first bring-up, power the LaunchPad from ICDI USB and share ground only.

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"

#define CONSOLE_UART_BASE UART0_BASE
#define FPGA_UART_BASE    UART1_BASE

#define CONSOLE_UART_BAUD 115200u
#define FPGA_UART_BAUD    115200u

#define LED_PORT_BASE GPIO_PORTF_BASE
#define LED_PIN       GPIO_PIN_2

static const uint16_t TELEM_PKT_SYNC = 0xA55Au;

static const uint8_t ST_OK = 0x00u;

static const uint8_t PKT_BRIDGE_HEARTBEAT = 0x50u;
static const uint8_t PKT_BRIDGE_RANGE_AGL = 0x51u;
static const uint8_t PKT_UNSUPPORTED_TEST = 0x7Eu;

static const uint16_t EXT_SRC_REAL = 1u << 0;
static const uint16_t EXT_SRC_TM4C_BRIDGE = 1u << 1;

static const uint32_t HEARTBEAT_PERIOD_MS = 100u;
static const uint32_t RANGE_PERIOD_MS = 50u;

static volatile uint32_t g_ms = 0u;
static uint32_t g_sys_clock_hz = 0u;

static uint16_t heartbeat_seq = 0x2000u;
static uint16_t range_seq = 0x0100u;
static uint32_t next_heartbeat_ms = 0u;
static uint32_t next_range_ms = 0u;

static bool heartbeat_enabled = true;
static bool range_enabled = true;
static bool corrupt_next_range = false;
static bool corrupt_next_heartbeat = false;
static bool out_of_range_next = false;
static bool low_confidence_next = false;
static bool unsupported_next = false;

static uint16_t range_height_cm = 185u;
static uint16_t range_confidence = 95u;

void
SysTickIntHandler(void)
{
    g_ms++;
}

static uint32_t
now_ms(void)
{
    return g_ms;
}

static void
console_putc(char c)
{
    if (c == '\n') {
        UARTCharPut(CONSOLE_UART_BASE, '\r');
    }
    UARTCharPut(CONSOLE_UART_BASE, c);
}

static void
console_puts(const char *s)
{
    while (*s != '\0') {
        console_putc(*s++);
    }
}

static void
console_put_u32(uint32_t v)
{
    char buf[11];
    uint32_t i = 0u;

    if (v == 0u) {
        console_putc('0');
        return;
    }

    while ((v != 0u) && (i < sizeof(buf))) {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (i != 0u) {
        console_putc(buf[--i]);
    }
}

static void
console_put_bool(const char *name, bool value)
{
    console_puts(name);
    console_puts(value ? "=1\n" : "=0\n");
}

static void
fpga_uart_put_u8(uint8_t v)
{
    UARTCharPut(FPGA_UART_BASE, (uint32_t)v);
}

static void
fpga_uart_put_u16(uint16_t v)
{
    fpga_uart_put_u8((uint8_t)(v >> 8));
    fpga_uart_put_u8((uint8_t)v);
}

static void
fpga_uart_put_u32(uint32_t v)
{
    fpga_uart_put_u8((uint8_t)(v >> 24));
    fpga_uart_put_u8((uint8_t)(v >> 16));
    fpga_uart_put_u8((uint8_t)(v >> 8));
    fpga_uart_put_u8((uint8_t)v);
}

static void
fpga_uart_put_u48(uint64_t v)
{
    fpga_uart_put_u8((uint8_t)(v >> 40));
    fpga_uart_put_u8((uint8_t)(v >> 32));
    fpga_uart_put_u8((uint8_t)(v >> 24));
    fpga_uart_put_u8((uint8_t)(v >> 16));
    fpga_uart_put_u8((uint8_t)(v >> 8));
    fpga_uart_put_u8((uint8_t)v);
}

static uint16_t
checksum16(uint8_t type,
           uint8_t status,
           uint16_t seq,
           uint32_t timestamp_us,
           uint64_t payload,
           uint16_t aux,
           uint16_t source_flags)
{
    return (uint16_t)(TELEM_PKT_SYNC ^
                      ((uint16_t)status << 8 | type) ^
                      seq ^
                      (uint16_t)(timestamp_us >> 16) ^
                      (uint16_t)timestamp_us ^
                      (uint16_t)(payload >> 32) ^
                      (uint16_t)(payload >> 16) ^
                      (uint16_t)payload ^
                      aux ^
                      source_flags);
}

static void
send_frame(uint8_t type,
           uint8_t status,
           uint16_t seq,
           uint32_t timestamp_us,
           uint64_t payload,
           uint16_t aux,
           uint16_t source_flags,
           bool corrupt_checksum)
{
    uint16_t sum = checksum16(type, status, seq, timestamp_us, payload,
                              aux, source_flags);
    if (corrupt_checksum) {
        sum ^= 0x0001u;
    }

    fpga_uart_put_u8(0xA5u);
    fpga_uart_put_u8(0x5Au);
    fpga_uart_put_u8(type);
    fpga_uart_put_u8(status);
    fpga_uart_put_u16(seq);
    fpga_uart_put_u32(timestamp_us);
    fpga_uart_put_u48(payload);
    fpga_uart_put_u16(aux);
    fpga_uart_put_u16(source_flags);
    fpga_uart_put_u16(sum);
}

static void
send_heartbeat(uint32_t timestamp_us)
{
    send_frame(PKT_BRIDGE_HEARTBEAT,
               ST_OK,
               heartbeat_seq++,
               timestamp_us,
               0u,
               0xCAFEu,
               EXT_SRC_TM4C_BRIDGE,
               corrupt_next_heartbeat);
    corrupt_next_heartbeat = false;
}

static void
send_range(uint32_t timestamp_us)
{
    const uint16_t height = out_of_range_next ? 12000u : range_height_cm;
    const uint16_t confidence = low_confidence_next ? 0u : range_confidence;
    const uint16_t raw_detail = (uint16_t)((timestamp_us >> 8) ^ range_seq);
    const uint64_t payload =
        ((uint64_t)height << 32) |
        ((uint64_t)confidence << 16) |
        raw_detail;

    send_frame(PKT_BRIDGE_RANGE_AGL,
               ST_OK,
               range_seq++,
               timestamp_us,
               payload,
               0x3333u,
               EXT_SRC_REAL | EXT_SRC_TM4C_BRIDGE,
               corrupt_next_range);

    corrupt_next_range = false;
    out_of_range_next = false;
    low_confidence_next = false;
}

static void
send_unsupported(uint32_t timestamp_us)
{
    send_frame(PKT_UNSUPPORTED_TEST,
               ST_OK,
               range_seq++,
               timestamp_us,
               0x000100020003ull,
               0x7E7Eu,
               EXT_SRC_TM4C_BRIDGE,
               false);
}

static bool
due_ms(uint32_t now, uint32_t *next, uint32_t period)
{
    if ((int32_t)(now - *next) >= 0) {
        do {
            *next += period;
        } while ((int32_t)(now - *next) >= 0);
        return true;
    }
    return false;
}

static void
print_help(void)
{
    console_puts("\nCaelumFusion TM4C123GXL bridge producer\n");
    console_puts("UART1 PC5/J4.05 emits FPGA fixed packets at 115200 8N1.\n");
    console_puts("Commands over ICDI virtual COM UART0:\n");
    console_puts("  ?  help\n");
    console_puts("  h  toggle heartbeat frames\n");
    console_puts("  r  toggle range frames\n");
    console_puts("  c  corrupt next range checksum\n");
    console_puts("  b  corrupt next heartbeat checksum\n");
    console_puts("  o  send next range out of FPGA limit\n");
    console_puts("  l  send next range with low confidence\n");
    console_puts("  u  send one unsupported packet type\n");
    console_puts("  +  increase simulated height\n");
    console_puts("  -  decrease simulated height\n");
}

static void
handle_console_command(int32_t c)
{
    switch ((char)c) {
    case '?':
        print_help();
        break;
    case 'h':
        heartbeat_enabled = !heartbeat_enabled;
        console_put_bool("heartbeat_enabled", heartbeat_enabled);
        break;
    case 'r':
        range_enabled = !range_enabled;
        console_put_bool("range_enabled", range_enabled);
        break;
    case 'c':
        corrupt_next_range = true;
        console_puts("next range checksum will be corrupt\n");
        break;
    case 'b':
        corrupt_next_heartbeat = true;
        console_puts("next heartbeat checksum will be corrupt\n");
        break;
    case 'o':
        out_of_range_next = true;
        console_puts("next range height will be out of FPGA range\n");
        break;
    case 'l':
        low_confidence_next = true;
        console_puts("next range confidence will be low\n");
        break;
    case 'u':
        unsupported_next = true;
        console_puts("one unsupported packet will be sent\n");
        break;
    case '+':
        if (range_height_cm < 9900u) {
            range_height_cm += 5u;
        }
        console_puts("range_height_cm=");
        console_put_u32(range_height_cm);
        console_puts("\n");
        break;
    case '-':
        if (range_height_cm >= 5u) {
            range_height_cm -= 5u;
        }
        console_puts("range_height_cm=");
        console_put_u32(range_height_cm);
        console_puts("\n");
        break;
    default:
        break;
    }
}

static void
init_clock(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 |
                   SYSCTL_USE_PLL |
                   SYSCTL_XTAL_16MHZ |
                   SYSCTL_OSC_MAIN);
    g_sys_clock_hz = SysCtlClockGet();
}

static void
wait_peripheral_ready(uint32_t periph)
{
    while (!SysCtlPeripheralReady(periph)) {
    }
}

static void
init_uarts(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    wait_peripheral_ready(SYSCTL_PERIPH_GPIOA);
    wait_peripheral_ready(SYSCTL_PERIPH_GPIOC);
    wait_peripheral_ready(SYSCTL_PERIPH_UART0);
    wait_peripheral_ready(SYSCTL_PERIPH_UART1);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(CONSOLE_UART_BASE, g_sys_clock_hz, CONSOLE_UART_BAUD,
                        UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);
    UARTConfigSetExpClk(FPGA_UART_BASE, g_sys_clock_hz, FPGA_UART_BAUD,
                        UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);
}

static void
init_led(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    wait_peripheral_ready(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(LED_PORT_BASE, LED_PIN);
    GPIOPinWrite(LED_PORT_BASE, LED_PIN, 0u);
}

static void
init_systick(void)
{
    SysTickPeriodSet(g_sys_clock_hz / 1000u);
    SysTickIntEnable();
    SysTickEnable();
    IntMasterEnable();
}

static void
toggle_led(void)
{
    const uint8_t current = (uint8_t)GPIOPinRead(LED_PORT_BASE, LED_PIN);
    GPIOPinWrite(LED_PORT_BASE, LED_PIN, current ^ LED_PIN);
}

int
main(void)
{
    init_clock();
    init_uarts();
    init_led();
    init_systick();

    next_heartbeat_ms = now_ms() + 10u;
    next_range_ms = now_ms() + 20u;

    print_help();

    for (;;) {
        while (UARTCharsAvail(CONSOLE_UART_BASE)) {
            handle_console_command(UARTCharGetNonBlocking(CONSOLE_UART_BASE));
        }

        const uint32_t ms = now_ms();
        const uint32_t timestamp_us = ms * 1000u;

        if (unsupported_next) {
            unsupported_next = false;
            send_unsupported(timestamp_us);
        }

        if (heartbeat_enabled &&
            due_ms(ms, &next_heartbeat_ms, HEARTBEAT_PERIOD_MS)) {
            send_heartbeat(timestamp_us);
            toggle_led();
        }

        if (range_enabled &&
            due_ms(ms, &next_range_ms, RANGE_PERIOD_MS)) {
            send_range(timestamp_us);
        }
    }
}
