/* Bootloader memory map. Mirrors fw/shared/src/flash_layout.rs.
 * RAM is shrunk by 16 bytes so the noinit magic word slot at 0x20004FF0
 * never collides with .bss/.stack. cortex-m-rt's link.x adds .vector_table,
 * .text, .rodata, .data, .bss, .uninit on top of these regions. */

MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 8K
    RAM   : ORIGIN = 0x20000000, LENGTH = 20K - 16
}
