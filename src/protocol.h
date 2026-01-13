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

// Handshake protocol
// Controller & Device Power ON
// 1) Devices power on
// 2) Devices send discovery query on DISCOVERY_REQUEST_ID, containing serial_no
// 3) Controller responds with serial_no and a valid id assignment on DISCOVERY_ASSIGN_ID
//      3a) Controller should map serial_no to id persistantly
//      3b) Controller should map serial_no/id to axis/control
// 4) Device reports INFO on device channel (DEVICE_BASE_ID + assigned_id)
// The device is now ready to respond to commands

// Controller sends commands to a device on the device ID channel using the Controller Command type
// The device responds on the same channel, using the DEVICE Command type (controller type + 1)

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

typedef enum {
    CONTROLLER_GET_INFO = 0,
    DEVICE_INFO,
    CONTROLLER_SET_POSITION, // Fixed point, 16 bit
    DEVICE_POSITION, // Fixed point, 16 bit
    CONTROLLER_GET_STATUS,
    DEVICE_STATUS, // Device status, position, motor fault, endstops
    CONTROLLER_GET_ANALOG,
    DEVICE_ANALOG, // Device ADC reads (2 x 16 bit)
    CONTROLLER_SET_SERVO,
    DEVICE_SERVO, // Device Servo values
    CONTROLLER_SET_LED,
    DEVICE_LED, // Device led values
    CONTROLLER_SET_MOTOR,
    DEVICE_MOTOR,
    CONTROLLER_SET_FOC,
    DEVICE_FOC,
    // Streams allow uart <-> can bridging
    CONTROLLER_STREAM_START, // Start streaming to device UART
    DEVICE_STREAM_START, // Start streaming from device UART
    CONTROLLER_STREAM_DATA, // Stream write (TX) data
    DEVICE_STREAM_DATA, // Stream read (RX) data
    CONTROLLER_ACK,
    DEVICE_ACK,
    CONTROLLER_START_FW_UPDATE,
    DEVICE_FW_UPDATE,
    CONTROLLER_NETWORK_RESET,
    DEVICE_NETWORK_RESET,
} command_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint32_t serial_no;
    uint8_t fw_ver;
    uint8_t hw_ver;
} device_info_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t position; // Precision is only 14 bits
} device_position_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t endstop0 : 1; // Endstop0 state
    uint8_t endstop1 : 1; // Endstop1 state
    uint8_t misc : 1; // Misc state
    uint8_t fault: 1; // DRV8313 fault signal
    uint8_t mag: 4; // MT6701 mag register
} device_status_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t a0;
    uint16_t a1;
} device_analog_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t srv0;
    uint16_t srv1;
} device_servo_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t stat;
    uint8_t sys;
} device_led_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t value; // Torque? Voltage?
    uint8_t rst : 1;
    uint8_t sleep: 1;
} device_motor_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t foc_1;
    uint8_t foc_2;
    uint8_t foc_3;
    uint8_t foc_en : 1;
} device_foc_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t sequence_id;
    uint8_t data[6];
} stream_data_t;

// Top level message structs
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t serial_no;
    uint8_t id_value; // Assign or previous ID
} discovery_message_t;

typedef struct __attribute__((packed, aligned(4))) {
    uint8_t cmd; // command_t
    union {
        device_info_t info_data;
        device_position_t position_data;
        device_status_t status_data;
        device_analog_t analog_data;
        device_servo_t servo_data;
        device_led_t led_data;
        device_motor_t motor_data;
        device_foc_t foc_data;
        stream_data_t stream_data;
    };
} device_message_t;

static_assert(sizeof(device_message_t) <= 8, "device_message_t overflows CAN packet");

// Command protocol: 1 byte of opcode,  1-7 bytes of data

#endif // PROTOCOL_H_
