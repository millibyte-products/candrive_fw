#ifndef CANDRIVE_COMMON_API_H
#define CANDRIVE_COMMON_API_H

/* Frozen-ABI function-pointer table provided by the common image.
 * Lives at exactly 0x08001000 (start of the COMMON flash region).
 *
 * Bootloader and application call into common code via:
 *
 *     COMMON_API->can_send(0x123, payload, 4);
 *
 * RULES (enforce when adding entries):
 *   - Entries are append-only. Never reorder. Never repurpose.
 *   - When the table grows, bump `version`. Old callers reading new
 *     fields must check `version` first.
 *   - Functions in common MUST be reentrant and stateless (no .data,
 *     no .bss). State lives in peripherals or caller-supplied buffers.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define COMMON_API_BASE     0x08001000U
#define COMMON_API_MAGIC    0x56524443U   /* 'CDRV' little-endian */
#define COMMON_API_VERSION  1U

 /* CAN frame (mirror of bxCAN mailbox payload, kept simple) */
typedef struct {
    uint32_t id;          /* 11-bit standard ID, lower 11 bits */
    uint8_t  len;         /* 0..8 */
    uint8_t  rtr;         /* 0 = data, 1 = remote */
    uint8_t  _pad[2];
    uint8_t  data[8];
} can_frame_t;

/* CAN filter spec — phase 1 only supports 16-bit list-mode filters. */
typedef struct {
    uint8_t  bank;        /* 0..13 */
    uint16_t id1;         /* standard ID #1 */
    uint16_t id2;         /* standard ID #2 (use same as id1 to disable) */
} can_filter_t;

typedef struct common_api {
    /* --- v1 header --- */
    uint32_t magic;                       /* COMMON_API_MAGIC */
    uint16_t version;                     /* COMMON_API_VERSION */
    uint16_t flags;                       /* reserved, must be 0 */

    /* --- v1 entries (append-only) --- */

    /* Busy-wait microsecond delay using DWT cycle counter. */
    void     (*delay_us)(uint32_t us);

    /* USART3 (PB10/PB11) debug @ 115200 8N1. Non-blocking on TX-empty. */
    int      (*usart_init)(uint32_t baud);
    void     (*usart_putc)(char c);
    void     (*usart_write)(const void* buf, size_t len);

    /* bxCAN1 — PA11(RX)/PA12(TX), 1 Mbit/s by default. */
    int      (*can_init)(uint32_t bitrate);
    int      (*can_set_filter)(const can_filter_t* spec);
    int      (*can_send)(const can_frame_t* frame);  /* 1=ok, 0=no mailbox */
    int      (*can_recv)(can_frame_t* out);          /* 1=msg, 0=empty */

    /* STM32 CRC peripheral (poly = IEEE 802.3, MSB-first, no reflection). */
    void     (*crc32_reset)(void);
    uint32_t(*crc32_update)(const void* buf, size_t len);
    uint32_t(*crc32_compute)(const void* buf, size_t len);

    /* Flash erase/program. Refuses to touch bootloader (0x08000000-0x08000FFF)
     * or common (0x08001000-0x08001FFF) regions; protects against self-foot-shooting.
     * Returns 0 on success, negative on error. */
    int      (*flash_unlock)(void);
    void     (*flash_lock)(void);
    int      (*flash_erase_page)(uint32_t addr);
    int      (*flash_program)(uint32_t addr, const void* data, size_t len);

} common_api_t;

/* Single canonical pointer to the API table. */
#define COMMON_API ((const common_api_t *)COMMON_API_BASE)

/* Verify the table at runtime before calling into it. Cheap. */
static inline bool common_api_valid(void)
{
    return COMMON_API->magic == COMMON_API_MAGIC &&
        COMMON_API->version >= COMMON_API_VERSION;
}

#endif /* CANDRIVE_COMMON_API_H */
