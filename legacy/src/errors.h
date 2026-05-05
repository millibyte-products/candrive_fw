#ifndef _ERRORS_H_
#define _ERRORS_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#define ERROR(code, message) (code | (message << 8))

#define ERROR_INVALID_COMMAND (0x01)
#define ERROR_INVALID_ARGUMENT (0x02)
#define ERROR_INVALID_DEVICE_ID (0x03)
#define ERROR_INVALID_LED_INDEX (0x04)
#define ERROR_STREAM_INACTIVE (0x05)
#define ERROR_INVALID_SEQUENCE (0x06)
#define ERROR_BUSY (0x07)
#define ERROR_NOT_IMPLEMENTED (0x08)
#define ERROR_UNKNOWN_STATE (0x09)
#define ERROR_WRITE_FAILED (0x0A)

#endif // _ERRORS_H_