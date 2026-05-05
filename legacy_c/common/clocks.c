/* Clock tree setup — bare LL register bashing, no HAL.
 * This file is linked into bootloader.elf and app.elf only — common
 * uses delay.c (no globals) for timing.
 */

#include "clocks.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx.h"

#define HCLK_HZ   64000000U
#define PCLK1_HZ  32000000U   /* HCLK / 2 */
#define PCLK2_HZ  64000000U   /* HCLK / 1 */

void clocks_init_64mhz(void)
{
    /* 1. Switch to HSI so we can safely reconfigure PLL. */
    LL_RCC_HSI_Enable();
    while (!LL_RCC_HSI_IsReady()) { /* wait */ }
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) { }

    /* 2. Two flash wait states needed above 48 MHz. Enable prefetch. */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    LL_FLASH_EnablePrefetch();

    /* 3. Configure prescalers (AHB/1, APB1/2, APB2/1, ADC/4). */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_4);

    /* 4. PLL: HSI/2 × 16 = 64 MHz. */
    LL_RCC_PLL_Disable();
    while (LL_RCC_PLL_IsReady()) { }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
    LL_RCC_PLL_Enable();
    while (!LL_RCC_PLL_IsReady()) { }

    /* 5. Switch SYSCLK -> PLL. */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) { }

    /* 6. CMSIS bookkeeping. */
    LL_SetSystemCoreClock(HCLK_HZ);

    /* 7. Enable DWT cycle counter for delay_us. Cortex-M3 always has DWT. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t clocks_hclk(void)  { return HCLK_HZ;  }
uint32_t clocks_pclk1(void) { return PCLK1_HZ; }
uint32_t clocks_pclk2(void) { return PCLK2_HZ; }
