#include "user_store.h"
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

bool user_store_write(uint32_t offset, uint32_t *data, uint32_t word_count)
{
    if (offset + word_count * sizeof(uint32_t) > USER_STORE_SIZE)
    {
        return false;
    }
    bool op_success = true;
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    for (uint32_t word_index = 0; word_index < word_count; word_index++)
    {
        // Write 4 bytes to the start of flash. Repeat these calls to write more.
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, static_cast<uint32_t>(USER_STORE_BASE_ADDRESS) + offset + (word_index * sizeof(uint32_t)), data[word_index]) != HAL_OK)
        {
            op_success = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    return op_success;
}

bool user_store_read(uint32_t offset, uint32_t *data, uint32_t word_count)
{
    if (offset + word_count * sizeof(uint32_t) > USER_STORE_SIZE)
    {
        return false;
    }
    for (uint32_t word_index = 0; word_index < word_count; word_index++)
    {
        // Write 4 bytes to the start of flash. Repeat these calls to write more.
        data[word_index] = *reinterpret_cast<uint32_t *>(USER_STORE_BASE_ADDRESS) + offset + (word_index * sizeof(uint32_t));
    }
    return true;
}

bool user_store_erase()
{
    bool op_status = true;
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    uint32_t page_address = (USER_STORE_BASE_ADDRESS - FLASH_ORIGIN) / PAGE_SIZE;
    FLASH_EraseInitTypeDef erase_cfg;
    erase_cfg.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_cfg.PageAddress = page_address;
    erase_cfg.NbPages = 1;
    // Erase the entire flash
    if (HAL_FLASHEx_Erase(&erase_cfg, FLASH_TYPEERASE_PAGES) != HAL_OK)
    {
        op_status = false;
    }
    HAL_FLASH_Lock();
    return op_status;
}
