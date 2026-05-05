/* Application memory map. Same RAM layout as the bootloader so the noinit
 * magic word at 0x20004FF0 is shared across resets. FLASH starts after
 * bootloader (8K) + common (4K) + user_store (1K) = 0x08003400. */

MEMORY
{
    FLASH : ORIGIN = 0x08003400, LENGTH = 64K - 8K - 4K - 1K
    RAM   : ORIGIN = 0x20000000, LENGTH = 20K - 16
}
