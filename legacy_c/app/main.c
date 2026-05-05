/* candrive application — phase 1 stub.
 *
 * The point of this file in phase 1 is to prove the cross-image
 * architecture works end-to-end:
 *
 *   - We're linked at 0x08002400 (after bootloader + common + user store).
 *   - We re-point VTOR at our own vector table.
 *   - We call into common at 0x08001000 via COMMON_API for USART, CAN,
 *     CRC, flash. No code duplication for those subsystems.
 *
 * Phase 2 will pull in the real motor / FOC / protocol logic.
 */

#include "common_api.h"
#include "boot_magic.h"
#include "clocks.h"
#include "delay.h"
#include "flash.h"   /* for APP_BASE constant only */

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx.h"

#include <stdint.h>

/* Same noinit slot as the bootloader — each image has its own definition
 * but they land at the same physical RAM address (0x20004FF0). */
uint32_t boot_magic __attribute__((section(".noinit")));

#define LED_SYS_PIN    LL_GPIO_PIN_4   /* PB4 */
#define LED_STAT_PIN   LL_GPIO_PIN_5   /* PB5 */

static void led_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB |
                             LL_APB2_GRP1_PERIPH_AFIO);
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    LL_GPIO_InitTypeDef io = {0};
    io.Pin        = LED_SYS_PIN | LED_STAT_PIN;
    io.Mode       = LL_GPIO_MODE_OUTPUT;
    io.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &io);
}

int main(void)
{
    /* Make sure VTOR points at our vector table — the bootloader sets
     * this before jumping, but if we're flashed and run standalone
     * we still want it correct. */
    SCB->VTOR = APP_BASE;

    clocks_init_64mhz();
    boot_magic = BOOT_MAGIC_NORMAL;

    if (!common_api_valid()) {
        /* No common image — we can't talk on CAN or print debug.
         * Best we can do is a slow distress blink. */
        led_init();
        for (;;) {
            LL_GPIO_TogglePin(GPIOB, LED_SYS_PIN);
            delay_ms(1000);
        }
    }

    COMMON_API->usart_init(115200U);
    COMMON_API->usart_write("\r\n[app] candrive app v0.1 (phase 1)\r\n", 37);

    led_init();

    for (;;) {
        LL_GPIO_TogglePin(GPIOB, LED_STAT_PIN);
        COMMON_API->delay_us(500000U);
    }
}
