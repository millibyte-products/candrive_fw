#ifndef _FLASH_H_
#define _FLASH_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <stdint.h>

extern "C" {
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
}

// Memory layout from linker symbols
extern const uintptr_t USER_STORE_BASE_ADDRESS;
extern const uint32_t USER_STORE_SIZE;
extern const uint32_t PAGE_SIZE;
extern const uintptr_t FLASH_ORIGIN;

typedef struct
{
    uint32_t magic;
    uint32_t length;
    uint32_t crc;
} flash_header_t;

typedef struct
{
    // Serial number assignment
    uint32_t serial_no;
    // Hardware MFG info
    uint8_t hw_ver_major;
    uint8_t hw_ver_minor;
    uint8_t hw_ver_patch;
    uint8_t hw_revision;
    // Approximate date of hardware manufacture
    // Uinx timestamp
    uint64_t mfg_date;

} hardware_info_t;

typedef struct
{
    uint8_t public_key[16];
    uint8_t device_key[16];
} device_key_t;

typedef struct
{
    flash_header_t header;
    hardware_info_t hw_info;
    device_key_t keys;
} user_store_t;

class Flash
{
private:
    bool lock_pending;
public:
    Flash()
    {
       lock_pending = false;
    }

    static Flash* get_instance()
    {
        static Flash instance;
        return &instance;
    }

    bool write(uint32_t offset, uint32_t* data, uint32_t word_count)
    {
        if (offset + word_count * sizeof(uint32_t) > USER_STORE_SIZE)
        {
            return false;
        }
        bool op_success = true;
        lock_pending = false;
        if (HAL_FLASH_Unlock() != HAL_OK)
        {
            return false;
        }
        for (uint32_t word_index = 0; word_index < word_count; word_index++)
        {
            // Write 4 bytes to the start of flash. Repeat these calls to write more.
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, static_cast<uint32_t>(USER_STORE_BASE_ADDRESS) + offset + (word_index * sizeof(uint32_t)), data[word_index]) != HAL_OK) {
                op_success = false;
                break;
            }
        }
        HAL_FLASH_Lock();
        return op_success;
    }

    bool read(uint32_t offset, uint32_t* data, uint32_t word_count)
    {
        if (offset + word_count * sizeof(uint32_t) > USER_STORE_SIZE)
        {
            return false;
        }
        lock_pending = false;
        for (uint32_t word_index = 0; word_index < word_count; word_index++)
        {
            // Write 4 bytes to the start of flash. Repeat these calls to write more.
            data[word_index] = *reinterpret_cast<uint32_t*>(USER_STORE_BASE_ADDRESS) + offset + (word_index * sizeof(uint32_t));
        }
        return true;
    }

    bool erase()
    {
        bool op_status = true;
        if (HAL_FLASH_Unlock() != HAL_OK)
        {
            return false;
        }
        uint32_t page_address = (USER_STORE_BASE_ADDRESS - FLASH_ORIGIN ) / PAGE_SIZE;
        FLASH_EraseInitTypeDef erase_cfg;
        erase_cfg.TypeErase = FLASH_TYPEERASE_PAGES;
        erase_cfg.PageAddress = page_address;
        erase_cfg.NbPages = 1;
        // Erase the entire flash
        if (HAL_FLASHEx_Erase(&erase_cfg, FLASH_TYPEERASE_PAGES) != HAL_OK) {
            op_status = false;
        }
        HAL_FLASH_Lock();
        return op_status;
    }

};

#endif // _FLASH_H_
