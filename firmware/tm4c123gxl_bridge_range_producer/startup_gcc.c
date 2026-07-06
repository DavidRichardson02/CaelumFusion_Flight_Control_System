// Minimal GCC startup for TM4C123GH6PM. This project only enables SysTick; all
// other unexpected interrupts land in IntDefaultHandler for obvious debugging.

#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;

extern int main(void);
extern void SysTickIntHandler(void);

void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))(&_estack),
    ResetISR,
    NmiSR,
    FaultISR,
    IntDefaultHandler,
    IntDefaultHandler,
    IntDefaultHandler,
    0,
    0,
    0,
    0,
    IntDefaultHandler,
    IntDefaultHandler,
    0,
    IntDefaultHandler,
    SysTickIntHandler
};

void
ResetISR(void)
{
    uint32_t *src;
    uint32_t *dst;

    src = &_etext;
    for (dst = &_data; dst < &_edata;) {
        *dst++ = *src++;
    }

    for (dst = &_bss; dst < &_ebss;) {
        *dst++ = 0u;
    }

    (void)main();

    while (1) {
    }
}

static void
NmiSR(void)
{
    while (1) {
    }
}

static void
FaultISR(void)
{
    while (1) {
    }
}

static void
IntDefaultHandler(void)
{
    while (1) {
    }
}
