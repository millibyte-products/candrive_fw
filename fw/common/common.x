/* Common-image linker script.
 *
 * The common image is just a code blob with a function-pointer table at
 * the very front. It is never entered at reset — the bootloader and app
 * jump into individual functions via the API table.
 *
 * Layout invariants enforced here:
 *   - The .api_table section is at offset 0 of the COMMON region (= 0x08001000).
 *   - The image contains NO .data and NO .bss (link-time ASSERT) so all
 *     functions are stateless and re-entrant — peripheral state lives in
 *     hardware registers.
 */

ENTRY(common_api_v1)   /* arbitrary anchor; never executed at reset */

MEMORY
{
    COMMON (rx) : ORIGIN = 0x08002000, LENGTH = 4K
}

SECTIONS
{
    /* The API table MUST be at offset 0 of COMMON — the bootloader/app
     * read it as `*(0x08001000 as *const CommonApi)`. */
    .api_table ORIGIN(COMMON) :
    {
        KEEP(*(.api_table))
    } > COMMON

    .text :
    {
        . = ALIGN(4);
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.glue_7) *(.glue_7t)
        . = ALIGN(4);
    } > COMMON

    .data : { *(.data .data.*) } > COMMON
    .bss  : { *(.bss .bss.*) *(COMMON) } > COMMON

    /DISCARD/ :
    {
        *(.ARM.exidx*)
        *(.ARM.extab*)
        *(.ARM.attributes)
        *(.eh_frame*)
        *(.note.*)
        *(.comment)
    }

    ASSERT(SIZEOF(.bss)  == 0, "common: .bss must be empty (no statics allowed)")
    ASSERT(SIZEOF(.data) == 0, "common: .data must be empty (no init'd statics allowed)")
}
