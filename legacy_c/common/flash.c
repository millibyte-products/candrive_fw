/* STM32F1 embedded flash programming — bare register access.
 *
 * Operations:
 *   - unlock: write KEY1, KEY2 to FLASH->KEYR
 *   - erase 1KB page: PER=1, AR=addr, STRT=1, wait BSY
 *   - program 16-bit halfword: PG=1, write halfword, wait BSY
 *   - lock: set LOCK
 *
 * Self-protection: refuses any write that overlaps bootloader (4 KiB)
 * or common (4 KiB). User store + app are writable.
 */

#include "flash.h"
#include "stm32f1xx.h"

#define FLASH_FPEC_KEY1  0x45670123U
#define FLASH_FPEC_KEY2  0xCDEF89ABU

/* Spin until BSY clears. Returns 0 if ok, -1 on PGERR/WRPRTERR. */
static int wait_done(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { }
    if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) {
        FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
        return -1;
    }
    FLASH->SR = FLASH_SR_EOP;
    return 0;
}

static int region_writable(uint32_t addr, size_t len)
{
    const uint32_t end = addr + len;
    if (addr < FLASH_BASE_ADDR) return 0;
    if (end < addr) return 0;                       /* overflow */
    if (addr < (BOOTLOADER_BASE + BOOTLOADER_SIZE) && end > BOOTLOADER_BASE) return 0;
    if (addr < (COMMON_BASE     + COMMON_SIZE)     && end > COMMON_BASE)     return 0;
    if (end > (APP_BASE + APP_SIZE)) return 0;
    return 1;
}

int flash_unlock(void)
{
    if (!(FLASH->CR & FLASH_CR_LOCK)) return 0;     /* already unlocked */
    FLASH->KEYR = FLASH_FPEC_KEY1;
    FLASH->KEYR = FLASH_FPEC_KEY2;
    return (FLASH->CR & FLASH_CR_LOCK) ? -1 : 0;
}

void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

int flash_erase_page(uint32_t addr)
{
    if (!region_writable(addr, FLASH_PAGE_SIZE)) return -2;
    if (addr & (FLASH_PAGE_SIZE - 1U))           return -3;

    while (FLASH->SR & FLASH_SR_BSY) { }
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = addr;
    FLASH->CR |= FLASH_CR_STRT;
    int rc = wait_done();
    FLASH->CR &= ~FLASH_CR_PER;
    return rc;
}

int flash_program(uint32_t addr, const void *data, size_t len)
{
    if (!region_writable(addr, len)) return -2;
    if (len & 1U)                    return -3;     /* halfword granular */
    if (addr & 1U)                   return -3;

    const uint16_t *src = (const uint16_t *)data;
    const size_t halfwords = len / 2U;

    while (FLASH->SR & FLASH_SR_BSY) { }
    FLASH->CR |= FLASH_CR_PG;
    for (size_t i = 0; i < halfwords; i++) {
        *(volatile uint16_t *)(addr + i * 2U) = src[i];
        if (wait_done()) {
            FLASH->CR &= ~FLASH_CR_PG;
            return -1;
        }
    }
    FLASH->CR &= ~FLASH_CR_PG;
    return 0;
}
