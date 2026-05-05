#ifndef _CAN_INTERFACE_H_
#define _CAN_INTERFACE_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <STM32_CAN.h>
#include "protocol.h"

#define CAN_FRAME_MAX_DATA_LENGTH (8)

void can_init();
void can_set_device_filter(int16_t device_id);
void can_read();
void can_write(uint32_t id, uint8_t *buffer, size_t length);
// Frame builder api
void can_start_frame();
void can_end_frame(uint32_t id);
void can_write_frame(uint8_t *buffer, size_t length);
void can_write_frame(uint8_t data);
void can_write_frame(uint16_t data);
void can_write_frame(uint32_t data);
uint8_t can_frame_length();

#endif // _CAN_INTERFACE_H_
