/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <STM32_CAN.h>
#include "protocol.h"
#include "can_interface.h"
#include "device.h"

static STM32_CAN driver(CAN1);
static uint8_t buffer[CAN_FRAME_MAX_DATA_LENGTH];
static uint8_t *write_ptr = buffer;

void set_filter(uint8_t message_channel)
{
    // Always listen to control messages on channel 0
    driver.setFilterDualID(0, CONTROL_ID, message_channel, STD, STD);
    // Dynamically set between discovery and device message channels
    driver.setFilter(0, true);
}

void can_init()
{
    driver.setBaudRate(1000000);
    driver.begin();
    // Filter for controller messages
    set_filter(DISCOVERY_ASSIGN_ID);
}

void can_start_frame()
{
    Serial3.printf("Starting frame\n");
    write_ptr = buffer;
}

void can_write_frame(uint8_t *buffer, size_t length)
{
    if ((can_frame_length() + length) > CAN_FRAME_MAX_DATA_LENGTH)
    {
        return;
    }
    memcpy(write_ptr, buffer, length);
    write_ptr += length;
}

void can_write_frame(uint8_t data)
{
    if ((write_ptr - buffer) + sizeof(uint8_t) > CAN_FRAME_MAX_DATA_LENGTH)
    {
        return;
    }
    *write_ptr++ = data;
}

void can_write_frame(uint16_t data)
{
    if ((write_ptr - buffer) + sizeof(uint16_t) > CAN_FRAME_MAX_DATA_LENGTH)
    {
        return;
    }
    *write_ptr++ = data >> 8;
    *write_ptr++ = data & 0xFF;
}

void can_write_frame(uint32_t data)
{
    if ((write_ptr - buffer) + sizeof(uint32_t) > CAN_FRAME_MAX_DATA_LENGTH)
    {
        return;
    }
    *write_ptr++ = data >> 24;
    *write_ptr++ = (data >> 16) & 0xFF;
    *write_ptr++ = (data >> 8) & 0xFF;
    *write_ptr++ = data & 0xFF;
}

uint8_t can_frame_length()
{
    return (uint8_t)(write_ptr - buffer);
}

void can_set_device_filter(int16_t device_id)
{
    set_filter((uint8_t)device_id);
}

void can_read()
{
    CAN_message_t can_msg;
    if (driver.read(can_msg))
    {
        handle_message(can_msg);
    }
}

void can_write(uint32_t id, uint8_t *buffer, size_t length)
{
    if (buffer && length <= CAN_FRAME_MAX_DATA_LENGTH)
    {
        CAN_message_t msg;
        msg.id = id;
        msg.len = length;
        memcpy(msg.buf, buffer, length);
        driver.write(msg);
    }
}

void can_end_frame(uint32_t id)
{
    Serial3.printf("Writing %d bytes: [", can_frame_length());
    for (size_t i = 0; i < can_frame_length(); i++)
    {
        Serial3.printf("%02X ", buffer[i]);
    }
    Serial3.printf("]\n");
    can_write(id, buffer, can_frame_length());
    write_ptr = buffer;
}
