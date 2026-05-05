#ifndef CANDRIVE_BOOT_MAGIC_H
#define CANDRIVE_BOOT_MAGIC_H

/* Cross-reset signaling between bootloader and app via a noinit RAM word.
 * The linker places a single uint32_t at the top of RAM (0x20004FF0) so
 * the variable persists across a NVIC_SystemReset(). Each image owns its
 * own definition (they're separate ELFs); both place it in section .noinit
 * which the linker maps to MEMORY region NOINIT (see the linker scripts). */

#include <stdint.h>

#define BOOT_MAGIC_STAY_IN_BOOTLOADER   0xB007EDBAU   /* request: skip jump-to-app */
#define BOOT_MAGIC_NORMAL               0xA9999999U   /* set by app at start so a
                                                         later reset goes back to app */

extern uint32_t boot_magic;   /* defined per-image in main.c */

#endif /* CANDRIVE_BOOT_MAGIC_H */
