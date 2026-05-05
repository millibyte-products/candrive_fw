/* The frozen-ABI function pointer table that lives at 0x08001000. */
#include "common_api.h"
#include "delay.h"
#include "usart_dbg.h"
#include "bxcan.h"
#include "crc32.h"
#include "flash.h"

const common_api_t common_api_v1
    __attribute__((section(".api_table"), used)) = {
    .magic   = COMMON_API_MAGIC,
    .version = COMMON_API_VERSION,
    .flags   = 0,

    .delay_us       = delay_us,

    .usart_init     = usart_dbg_init,
    .usart_putc     = usart_dbg_putc,
    .usart_write    = usart_dbg_write,

    .can_init       = bxcan_init,
    .can_set_filter = bxcan_set_filter,
    .can_send       = bxcan_send,
    .can_recv       = bxcan_recv,

    .crc32_reset    = crc32_reset,
    .crc32_update   = crc32_update,
    .crc32_compute  = crc32_compute,

    .flash_unlock     = flash_unlock,
    .flash_lock       = flash_lock,
    .flash_erase_page = flash_erase_page,
    .flash_program    = flash_program,
};
