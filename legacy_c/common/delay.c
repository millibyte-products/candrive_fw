/* Cycle-counter busy-wait. Lives in common (no globals, no .data/.bss).
 *
 * Assumes the bootloader/app has already enabled DWT in their
 * clocks_init_64mhz() before any common code that delays runs.
 *
 * The HCLK frequency is hard-coded — common can't read SystemCoreClock
 * (would pull a .data symbol from CMSIS). If the clock tree ever
 * changes we update this constant in lockstep across all images.
 */

#include "delay.h"
#include "stm32f1xx.h"

#define HCLK_HZ_LOCKED   64000000U

void delay_us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = us * (HCLK_HZ_LOCKED / 1000000U);
    while ((DWT->CYCCNT - start) < cycles) { /* spin */ }
}

void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000U);
}
