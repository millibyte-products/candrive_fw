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

extern "C"
{
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
}

// Memory layout from linker symbols
#define USER_STORE_BASE_ADDRESS 0x08001000
#define USER_STORE_SIZE 0x0100
#define PAGE_SIZE 1024
#define FLASH_ORIGIN 0x08000000

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

bool user_store_write(uint32_t offset, uint32_t *data, uint32_t word_count);
bool user_store_read(uint32_t offset, uint32_t *data, uint32_t word_count);
bool user_store_erase();

#endif // _FLASH_H_
