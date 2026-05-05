/* Common code blob — pure functions only, no .data/.bss.
 * Lives at 0x08001000 with a frozen-ABI function-pointer table
 * (.api_table) at offset 0 so bootloader and app can call into it
 * via ((const struct common_api *)0x08001000).
 *
 * The .bss/.data assertion below catches accidental statics in
 * common code at link time.
 */
ENTRY(common_api_v1)   /* arbitrary; common is never entered at reset */

MEMORY
{
    COMMON (rx) : ORIGIN = 0x08001000, LENGTH = 4K
}

SECTIONS
{
    /* The API table MUST be at offset 0 of the COMMON region. */
    .api_table ORIGIN(COMMON) :
    {
        KEEP(*(.api_table))
    } > COMMON

    .text :
    {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        *(.glue_7) *(.glue_7t)
        . = ALIGN(4);
    } > COMMON

    .data : { *(.data*) } > COMMON
    .bss  : { *(.bss*) *(COMMON) } > COMMON

    /DISCARD/ :
    {
        *(.ARM.exidx*)
        *(.ARM.extab*)
        *(.ARM.attributes)
        *(.eh_frame*)
        *(.note*)
        *(.comment)
    }

    ASSERT(SIZEOF(.bss)  == 0, "common: must not contain .bss (no statics allowed)")
    ASSERT(SIZEOF(.data) == 0, "common: must not contain .data (no initialized statics allowed)")
}
