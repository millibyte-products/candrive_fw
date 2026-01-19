#include "device.h"

// Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
// You may use, distribute and modify this code under the
// terms of the CC BY-NC-SA license.
//
// You should have received a copy of the CC BY-NC-SA license with
// this file. If not, please visit : github.com/millibyte/candrive-fw

// EXTERNAL MACROS (set by build system)
// #define BUILD_COMMIT "unknown"
// #define BUILD_TS 1665072400
// #define VERSION_MAJOR 0
// #define VERSION_MINOR 9
// #define VERSION_PATCH 0

#include "build_info.h"

// Maps to upper 8 bits of CAN ID
static int16_t device_id = INVALID_DEVICE;
static reset_reason_t last_reason = RESET_REASON_UNKNOWN;

reset_reason_t get_reset_reason(void)
{
    reset_reason_t reset_reason = RESET_REASON_UNKNOWN;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_reason = RESET_REASON_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_reason = RESET_REASON_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_reason = RESET_REASON_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        // NVIC_SystemReset()
        reset_reason = RESET_REASON_SOFTWARE_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_reason = RESET_REASON_POWER_ON_POWER_DOWN_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_reason = RESET_REASON_EXTERNAL_RESET_PIN_RESET;
    }

    // Clear all the reset flags or else they will remain set during future
    // resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_reason;
}

void init_device()
{
    device_id = INVALID_DEVICE;
    last_reason = get_reset_reason();
}

int16_t get_device_id()
{
    return device_id;
}

void set_device_id(int16_t value)
{
    device_id = value;
}

uint32_t get_serial_no()
{
    // TODO read/write to OTP
    return 0;
}
uint8_t get_fw_major()
{
    // Set by build system
    return VERSION_MAJOR;
}

uint8_t get_fw_minor()
{
    // Set by build system
    return VERSION_MINOR;
}

uint8_t get_fw_patch()
{
    // Set by build system
    return VERSION_PATCH;
}

uint64_t get_build_ts()
{
    // Set by build system
    return BUILD_TIMESTAMP;
}

const uint8_t* get_build_commit()
{
    // Set by build system
    return GIT_SHA;
}

uint8_t get_protocol_ver()
{
    // Read from fw
    return PROTOCOL_VER;
}

uint8_t get_hw_ver()
{
    // Read from flash metadata
    return 0;
}

reset_reason_t get_last_reset_reason(void)
{
    return last_reason;
}