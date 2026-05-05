/* candrive bootloader.
 *
 * Phase 1 responsibilities (all that's wired in this image):
 *   1. clock the chip up to 64 MHz
 *   2. bring up USART3 debug
 *   3. decide whether to jump to the application based on a noinit
 *      magic word at 0x20004FF0 plus a sanity check of the app's
 *      vector table at 0x08002400
 *   4. if staying, blink the SYS LED (PB4) so a human can tell which
 *      image is running. CAN-driven firmware update lands in phase 2.
 *
 * The bootloader is its own ELF; common.elf is a separate image
 * programmed at 0x08001000 — we call into it through COMMON_API.
 */

#include "common_api.h"
#include "boot_magic.h"
#include "clocks.h"
#include "delay.h"
#include "usart_dbg.h"
#include "flash.h"

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx.h"

#include <stdint.h>
#include <stdbool.h>

/* Persistent across resets; see boot_magic.h. */
uint32_t boot_magic __attribute__((section(".noinit")));

#define LED_SYS_PORT   GPIOB
#define LED_SYS_PIN    LL_GPIO_PIN_4

static void led_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB |
                             LL_APB2_GRP1_PERIPH_AFIO);
    /* JTAG occupies PB3/PB4 by default — disable JTAG, keep SWD. */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    LL_GPIO_InitTypeDef io = {0};
    io.Pin        = LED_SYS_PIN;
    io.Mode       = LL_GPIO_MODE_OUTPUT;
    io.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(LED_SYS_PORT, &io);
}

static bool app_looks_valid(void)
{
    const uint32_t *vec = (const uint32_t *)APP_BASE;
    const uint32_t sp    = vec[0];
    const uint32_t reset = vec[1];

    /* SP must point into RAM. */
    if (sp < 0x20000000U || sp > 0x20005000U) return false;
    /* Reset vector must point into the app flash region with Thumb bit set. */
    if ((reset & 1U) == 0)                    return false;
    if (reset < APP_BASE || reset >= (APP_BASE + APP_SIZE)) return false;
    return true;
}

__attribute__((noreturn))
static void jump_to_app(void)
{
    const uint32_t *vec = (const uint32_t *)APP_BASE;
    const uint32_t sp           = vec[0];
    const uint32_t reset_handler = vec[1];

    /* Quiesce: clear pending IRQs, mask interrupts, point VTOR at app. */
    __disable_irq();
    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    SCB->VTOR = APP_BASE;
    __DSB();
    __ISB();

    __set_MSP(sp);
    __enable_irq();

    /* Tail-call into the app's reset handler — this never returns. */
    ((void (*)(void))reset_handler)();
    for (;;) { }
}

int main(void)
{
    clocks_init_64mhz();

    /* Common image must already be programmed at 0x08001000 — without
     * it we can't do much except blink. Validate the API table. */
    const bool common_ok = common_api_valid();

    if (common_ok) {
        COMMON_API->usart_init(115200U);
        COMMON_API->usart_write("\r\n[bl] candrive bootloader\r\n", 28);
    } else {
        /* Bring USART up locally as a fallback (this code is the same
         * as common's, just linked into the bootloader image too). */
        usart_dbg_init(115200U);
        usart_dbg_puts("\r\n[bl] WARN: common image missing/invalid\r\n");
    }

    led_init();

    const uint32_t magic = boot_magic;
    boot_magic = 0;   /* one-shot — clear so the next reset re-evaluates */

    if (magic != BOOT_MAGIC_STAY_IN_BOOTLOADER && app_looks_valid()) {
        if (common_ok) COMMON_API->usart_write("[bl] jumping to app\r\n", 21);
        jump_to_app();
    }

    /* Stay in bootloader: blink fast, await firmware update (phase 2). */
    if (common_ok) COMMON_API->usart_write("[bl] staying (no app or magic set)\r\n", 36);
    for (;;) {
        LL_GPIO_TogglePin(LED_SYS_PORT, LED_SYS_PIN);
        delay_ms(150);
    }
}
