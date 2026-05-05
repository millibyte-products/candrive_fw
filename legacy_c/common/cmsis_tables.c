/* CMSIS prescaler tables — stripped from system_stm32f1xx.c so we can
 * include them in common.elf without pulling in the writable
 * SystemCoreClock global. These are pure const, .rodata only. */

#include <stdint.h>

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U]  = {0, 0, 0, 0, 1, 2, 3, 4};
