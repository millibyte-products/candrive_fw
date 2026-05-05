#ifndef CANDRIVE_FLASH_H
#define CANDRIVE_FLASH_H

#include <stdint.h>
#include <stddef.h>

/* Flash layout (mirrored in linker scripts):
 *   0x08000000  bootloader  4K   protected
 *   0x08001000  common      4K   protected
 *   0x08002000  user_store  1K   writable
 *   0x08002400  application 55K  writable
 */
#define FLASH_BASE_ADDR        0x08000000U
#define FLASH_PAGE_SIZE        1024U

#define BOOTLOADER_BASE        0x08000000U
#define BOOTLOADER_SIZE        (4U * 1024U)
#define COMMON_BASE            0x08001000U
#define COMMON_SIZE            (4U * 1024U)
#define USER_STORE_BASE        0x08002000U
#define USER_STORE_SIZE        (1U * 1024U)
#define APP_BASE               0x08002400U
#define APP_SIZE               (55U * 1024U)

int  flash_unlock(void);
void flash_lock(void);

/* Erase a single 1KB page. Refuses bootloader/common region. */
int  flash_erase_page(uint32_t addr);

/* Program `len` bytes (must be even, halfword-aligned address).
 * Refuses bootloader/common region. */
int  flash_program(uint32_t addr, const void* data, size_t len);

#endif /* CANDRIVE_FLASH_H */
