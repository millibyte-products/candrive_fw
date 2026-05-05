#ifndef CANDRIVE_CLOCKS_H
#define CANDRIVE_CLOCKS_H

#include <stdint.h>

/* HSI ÷ 2 × 16 = 64 MHz HCLK, 32 MHz APB1, 64 MHz APB2.
 * Idempotent and safe to call early (uses HSI as transition source).
 * Also enables DWT cycle counter — required by delay_us in delay.h.
 */
void clocks_init_64mhz(void);

/* Frequency accessors (constants, since we lock the tree at boot). */
uint32_t clocks_hclk(void);
uint32_t clocks_pclk1(void);
uint32_t clocks_pclk2(void);

#endif /* CANDRIVE_CLOCKS_H */
