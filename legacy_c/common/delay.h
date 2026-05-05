#ifndef CANDRIVE_DELAY_H
#define CANDRIVE_DELAY_H

#include <stdint.h>

/* Cycle-counter busy-wait helpers. DWT must already be enabled
 * (clocks_init_64mhz does this). HCLK is assumed locked at 64 MHz. */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* CANDRIVE_DELAY_H */
