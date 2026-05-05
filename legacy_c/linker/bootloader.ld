/* Bootloader image — first 4 KiB of flash, full 20 KiB RAM minus the
 * shared "noinit" magic word region at the top of RAM (also reserved
 * by app.ld so the two halves communicate across reset).
 */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 4K
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 20K - 16
    NOINIT (rw)  : ORIGIN = 0x20004FF0, LENGTH = 16
}

/* Stack lives at the top of RAM, just below the noinit slot. */
_estack       = ORIGIN(NOINIT);
_min_stack    = 0x800;   /* 2 KiB minimum stack */

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
           "bootloader: RAM overflow (bss + min stack)")
}
