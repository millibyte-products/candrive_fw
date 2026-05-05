#ifndef _STREAM_HANDLER_H_
#define _STREAM_HANDLER_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <stdint.h>
#include <stddef.h>

#include "protocol.h"

void configure_stream(uint8_t stream_target, uint8_t flags);
void handle_stream_start(uint16_t stream_length, uint32_t checksum);
void handle_stream_data(uint8_t sequence_id, uint8_t *buffer, size_t length);

stream_target_t stream_target_get();

#endif // _STREAM_HANDLER_H_
