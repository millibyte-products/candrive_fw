#include "device.h"

static uint8_t device_id = 0;

uint8_t get_device_id()
{
    return device_id;
}

void set_device_id(uint8_t id)
{
    device_id = id;
}

uint32_t get_serial_no()
{
    // Read from flash metadata
    return 0;
}
uint8_t get_fw_ver()
{
    // Read from flash metadata
    return 1;
}
uint8_t get_protocol_ver()
{
    // Read from fw
    return 1;
}
uint8_t get_hw_ver()
{
    // Read from flash metadata
    return 0;
}
