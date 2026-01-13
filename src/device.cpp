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

static uint8_t device_id = 0;

const uint8_t GIT_SHA[21] PROGMEM = {
    BUILD_COMMIT, 0x00
};
const uint32_t FW_VER PROGMEM = VERSION_MAJOR << 16 | VERSION_MINOR << 8 | VERSION_PATCH;
const uint64_t BUILD_TIMESTAMP PROGMEM = BUILD_TS;
const uint8_t PROTOCOL_VER PROGMEM = 1;

uint8_t get_device_id()
{
    return device_id;
}

void set_device_id(uint8_t value)
{
    device_id = value;
}

uint32_t get_serial_no()
{
    // TODO read/write to OTP
    return 0;
}
uint32_t get_fw_ver()
{
    // Set by build system
    return FW_VER;
}

uint64_t get_build_ts()
{
    // Set by build system
    return BUILD_TIMESTAMP;
}

uint8_t* get_build_commit()
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
