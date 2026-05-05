#ifndef CANDRIVE_USART_DBG_H
#define CANDRIVE_USART_DBG_H

#include <stdint.h>
#include <stddef.h>

/* Debug UART on USART3 (PB10 = TX, PB11 = RX), 8N1. Polled. */

int  usart_dbg_init(uint32_t baud);
void usart_dbg_putc(char c);
void usart_dbg_write(const void* buf, size_t len);

/* Convenience wrapper that's only available where common's not the host
 * (i.e. bootloader/app sources). */
void usart_dbg_puts(const char* s);

#endif /* CANDRIVE_USART_DBG_H */
