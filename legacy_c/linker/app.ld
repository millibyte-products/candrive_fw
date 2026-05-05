/* Application image — starts at 0x08002400 (after bootloader 4K + common 4K
 * + user_store 1K). Same RAM layout as bootloader.ld so the noinit magic
 * word at 0x20004FF0 is shared across resets.
 */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08002400, LENGTH = 64K - 4K - 4K - 1K  /* 55 KiB */
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 20K - 16
    NOINIT (rw)  : ORIGIN = 0x20004FF0, LENGTH = 16
}

_estack    = ORIGIN(NOINIT);
_min_stack = 0x1000;   /* 4 KiB minimum stack for app */

SECTIONS
{
    .isr_vector ORIGIN(FLASH) :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        *(.glue_7) *(.glue_7t)
        *(.eh_frame*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .ARM.exidx : { *(.ARM.exidx*) } > FLASH
    .ARM.extab : { *(.ARM.extab*) } > FLASH

    .data : AT (_etext)
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .noinit (NOLOAD) :
    {
        KEEP(*(.noinit*))
    } > NOINIT

    /DISCARD/ : { *(.ARM.attributes) *(.note*) *(.comment) }

    ASSERT(_ebss + _min_stack <= ORIGIN(RAM) + LENGTH(RAM),
           "app: RAM overflow (bss + min stack)")
}
