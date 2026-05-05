#ifndef PROTOCOL_H_
#define PROTOCOL_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <stdint.h>
#include <stddef.h>
#include "can_interface.h"

// Handshake protocol
// Controller & Device Power ON
// 1) Devices power on
// 2) Devices send discovery query on DISCOVERY_REQUEST_ID, containing serial_no
// 3) Controller responds with serial_no and a valid id assignment on DISCOVERY_ASSIGN_ID
//      3a) Controller should map serial_no to id persistantly
//      3b) Controller should map serial_no/id to axis/control
// 4) Device reports INFO on device channel (DEVICE_BASE_ID + assigned_id)
// The device is now ready to respond to commands

// Controller sends commands to a device on the device ID channel setting the controller bit
// The device responds on the same channel, resetting the controller bit and omitting any data
// Device responds with ACK and echos the command on success
// Device responds with ERROR and error code on failure

// CAN IDs 0 through 7 are reserved
// The main controller/host should use ID 0
// Devices needing to be discovered should use ID 1
// Configured Devices should use IDs 8-2047

#define CONTROL_ID (0x00)
#define DISCOVERY_ASSIGN_ID (0x01)
#define DISCOVERY_REQUEST_ID (0x02)
// Devices use upper 8 bits of ID
#define DEVICE_BASE_ID (0x08)

#define DEVICE_HEADER_SIZE (2)

#define CONTROLLER_MESSAGE(x) (0x80 | x)
#define DEVICE_MESSAGE(x) (0x80 ^ x)

#define STREAM_FLAGS_READ (0x00)
#define STREAM_FLAGS_WRITE (0x01)

typedef enum
{
    CMD_NOOP = 0x00,
    CMD_ACK,
    CMD_ERROR,
    CMD_RESET,

    CMD_GET_INFO,     // Device factory + discoveryinfo
    CMD_GET_INFO_EXT, // Device runtime info

    CMD_GET_POSITION, // Fixed point, 16 bit
    CMD_SET_POSITION, // Fixed point, 16 bit

    CMD_GET_STATUS, // Internal status

    CMD_GET_ANALOG, // Analog IO pins

    CMD_GET_SERVO,
    CMD_SET_SERVO, // Servio IO pins

    CMD_GET_LED,
    CMD_SET_LED, // Device led values

    // Streams allow uart <-> can bridging
    CMD_STREAM_START, // Start streaming to device UART
    CMD_STREAM_READ,  // Stream write (TX) data
    CMD_STREAM_WRITE, // Stream read (RX) data
    CMD_FIRMWARE_UPDATE,
    CMD_ERASE_FACTORY,
    CMD_WRITE_FACTORY,
    CMD_NETWORK_RESET,
    CMD_REVOKE_CONFIG, // Revoke all configuration data
    // Messages cannot use the 8th bit (controller/device signal)
    INVALID_CMD = 0x7F,
} command_t;

typedef enum {
    LED_STAT = 0x01,
    LED_SYS = 0x02,
} led_address_t;

typedef enum
{
    SERVO_0 = 0x01,
    SERVO_1 = 0x02,
} servo_address_t;

typedef enum
{
    ENDSTOP_0 = 0x01,
    ENDSTOP_1 = 0x02,
} endstop_address_t;

typedef enum
{
    ERROR_NONE = 0x00,
    ERROR_INVALID_COMMAND = 0x01,
    ERROR_INVALID_DATA = 0x02,
    ERROR_NOT_IMPLEMENTED = 0x03,
    ERROR_DEVICE_FAULT = 0x04,
    ERROR_STREAM_FAILURE = 0x05,
} error_code_t;

typedef enum {
    STREAM_TARGET_USER_STORE = 0x00,
    STREAM_TARGET_FIRMWARE = 0x01,
    STREAM_TARGET_FILE = 0x02,
} stream_target_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint32_t serial_no;
    uint8_t fw_ver_major;
    uint8_t fw_ver_minor;
    uint8_t fw_ver_patch;
} info_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t flags; // reset reason lower 4 bits, fault flags upper 4 bits
    uint8_t temperature;
} info_ext_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t a0;
    uint16_t a1;
} analog_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t srv0;
    uint16_t srv1;
    uint8_t update_mask;
} servo_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t sys;
    uint8_t stat;
    uint8_t update_mask;
} led_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t value; // Torque? Voltage?
    uint8_t flags;
} motor_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t foc_1;
    uint8_t foc_2;
    uint8_t foc_3;
    uint8_t foc_en;
} foc_data_t;

typedef struct __attribute__((packed, aligned(1)))
{
    uint16_t stream_length;
    uint32_t checksum;
} stream_start_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t data[7];
} stream_data_t;

typedef struct __attribute__((packed, aligned(1)))
{
    uint32_t serial_no;
} revoke_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t error_code;
    uint32_t error_message;
} error_data_t;

// Top level message structs
typedef struct __attribute__((packed, aligned(1))) {
    uint32_t serial_no;
    uint8_t id_value; // Assign or previous ID
} discovery_data_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t command;
    uint8_t data[7]; // command specific data
} device_message_t;

static_assert(sizeof(device_message_t) <= 8, "device_message_t overflows CAN packet");

// Command protocol: 1 byte of opcode,  1-7 bytes of data

#define COMMAND_CONTROLLER_MASK (0x80)
#define COMMAND_CMD_MASK (0x7F)
#define MOTOR_RST_MASK (0x01)
#define MOTOR_SLEEP_MASK (0x02)
#define ENDSTOP0_MASK (0x01)
#define ENDSTOP1_MASK (0x02)
#define MISC_MASK (0x04)
#define FAULT_MASK (0x08)
#define MAG_MASK (0xF0)
#define RESET_REASON_MASK (0x0F)
#define FAULT_FLAGS_MASK (0xF0)

void send_discovery_query();

void handle_discovery(uint8_t *data, size_t length);
void handle_control(uint8_t *data, size_t length);
void handlne_broadcast(uint8_t *data, size_t length);
void handle_message(CAN_message_t can_msg);

#endif // PROTOCOL_H_
