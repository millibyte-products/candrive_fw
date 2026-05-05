/* STM32 CRC peripheral driver. Word-only input.
 *
 * STM32F1's CRC unit is fixed: poly 0x04C11DB7, MSB-first, init 0xFFFFFFFF,
 * no reflection on input or output, no XOR-out. That means it does NOT
 * match the IEEE 802.3 (Ethernet/zlib) CRC32 — which is reflected and
 * XOR-out 0xFFFFFFFF. Both ends (firmware + Rust host) must use the
 * same convention; we'll add a software reference impl in the Rust
 * tools when phase 4 (firmware update) lands.
 */

#include "crc32.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx.h"

static void ensure_clock(void)
{
    /* Idempotent — calling it on every entry costs one register read. */
    if (!(RCC->AHBENR & RCC_AHBENR_CRCEN)) {
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
    }
}

void crc32_reset(void)
{
    ensure_clock();
    CRC->CR = CRC_CR_RESET;
}

uint32_t crc32_update(const void *buf, size_t len)
{
    ensure_clock();
    const uint32_t *p = (const uint32_t *)buf;
    /* Caller is responsible for 4-byte aligned len; truncate any tail. */
    const size_t words = len / 4U;
    for (size_t i = 0; i < words; i++) {
        CRC->DR = p[i];
    }
    return CRC->DR;
}

uint32_t crc32_compute(const void *buf, size_t len)
{
    crc32_reset();
    return crc32_update(buf, len);
}
