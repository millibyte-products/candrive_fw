/* Shim for ST's `stm32_assert.h` — the LL drivers include this name,
 * but the template lives next to them as stm32_assert_template.h.
 * We make assert_param a no-op (no USE_FULL_ASSERT). */

#ifndef STM32_ASSERT_H
#define STM32_ASSERT_H

#ifdef  USE_FULL_ASSERT
#include <stdint.h>
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#endif /* STM32_ASSERT_H */
