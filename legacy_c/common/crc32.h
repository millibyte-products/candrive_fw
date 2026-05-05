#ifndef CANDRIVE_CRC32_H
#define CANDRIVE_CRC32_H

#include <stdint.h>
#include <stddef.h>

/* STM32 CRC peripheral wrapper. Polynomial = 0x04C11DB7,
 * MSB-first, no reflection, init = 0xFFFFFFFF.  Word-aligned input only. */

void     crc32_reset(void);
uint32_t crc32_update(const void* buf, size_t len);    /* must be 4-byte aligned len */
uint32_t crc32_compute(const void* buf, size_t len);   /* reset + update */

#endif /* CANDRIVE_CRC32_H */
