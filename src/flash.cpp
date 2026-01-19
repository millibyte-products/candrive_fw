#include "flash.h"
#include <stdint.h>
#include <stddef.h>

// Memory layout from linker symbols
/*extern "C" {
extern const uintptr_t _PAGE_LENGTH;
extern const uintptr_t _FLASH_ORIGIN;
extern const uintptr_t _USER_STORE_ORIGIN;
extern const uintptr_t _USER_STORE_LENGTH;
}

const uint32_t PAGE_SIZE = (const uint32_t)(&_PAGE_LENGTH);
const uintptr_t FLASH_ORIGIN = (const uintptr_t)(&_FLASH_ORIGIN);
const uintptr_t USER_STORE_BASE_ADDRESS = (const uintptr_t)(&_USER_STORE_ORIGIN);
const uint32_t USER_STORE_SIZE = (const uint32_t)(&_USER_STORE_LENGTH);*/

const uint32_t PAGE_SIZE = 1024;
const uintptr_t FLASH_ORIGIN = 0x08000000;
const uintptr_t USER_STORE_BASE_ADDRESS = 0x00000;
const uint32_t USER_STORE_SIZE = 0;