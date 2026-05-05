#include "protocol.h"
#include "device.h"
#include "can_interface.h"
#include "motor.h"
#include "stream_handler.h"

#include <stdint.h>
#include <stddef.h>

void send_error_message(error_data_t *error_data)
{
    can_start_frame();
    can_write_frame((uint8_t)CMD_ERROR);
    can_write_frame((uint8_t *)error_data, sizeof(error_data_t));
    can_end_frame(device_id_get() + DEVICE_BASE_ID);
}

// Helper functions
void send_protocol_message(command_t cmd)
{
    // Implementation of sending a protocol message
    can_start_frame();
    // Copy data to write buffer
    switch (cmd)
    {
    case CMD_GET_INFO:
    {
        can_write_frame((uint8_t)cmd);
        info_data_t info = {
            .serial_no = serial_no_get(),
            .fw_ver_major = fw_major_get(),
            .fw_ver_minor = fw_minor_get(),
            .fw_ver_patch = fw_patch_get(),
        };
        can_write_frame((uint8_t *)&info, sizeof(info_data_t));
    }
    break;
    case CMD_GET_INFO_EXT:
    {
        can_write_frame((uint8_t)cmd);
        info_ext_data_t info = {
            .flags = (uint8_t)(reset_reason_get()) & RESET_REASON_MASK,
            .temperature = 0,
        };
        can_write_frame((uint8_t *)&info, sizeof(info_ext_data_t));
    }
    break;
    case CMD_GET_POSITION:
    {
        can_write_frame((uint8_t)cmd);
        can_write_frame((uint16_t)motor_get_angle());
    }
    break;
    case CMD_GET_STATUS:
    {
        can_write_frame((uint8_t)cmd);
        uint8_t flags =
            (0 ? digitalRead(ENDSTOP0) == LOW : 1) |
            (0 ? digitalRead(ENDSTOP1) == LOW : 1) << 1 |
            (0 ? digitalRead(MISC) == LOW : 1) << 2 |
            (0 ? digitalRead(M_NFAULT) == LOW : 1) << 3 |
            0 << 4;
        can_write_frame(flags);
    }
    break;
    case CMD_GET_ANALOG:
    {
        can_write_frame((uint8_t)cmd);
        analog_data_t analog_data = {
            .a0 = static_cast<uint16_t>(analogRead(A0)),
            .a1 = static_cast<uint16_t>(analogRead(A1)),
        };
        can_write_frame((uint8_t *)&analog_data, sizeof(analog_data_t));
    }
    break;
    case CMD_GET_SERVO:
    {
        can_write_frame((uint8_t)cmd);
        servo_data_t servo_data = {
            .srv0 = servo_get(SERVO_0),
            .srv1 = servo_get(SERVO_1),
            .update_mask = 0x3,
        };
        can_write_frame((uint8_t *)&servo_data, sizeof(servo_data_t));
    }
    break;
    case CMD_GET_LED:
    {
        can_write_frame((uint8_t)cmd);
        led_data_t led_data = {
            .sys = led_get(LED_SYS),
            .stat = led_get(LED_STAT),
            .update_mask = LED_SYS | LED_STAT,
        };
        can_write_frame((uint8_t *)&led_data, sizeof(led_data_t));
    }
    break;
    case CMD_GET_MOTOR:
    {
        can_write_frame((uint8_t)cmd);
        motor_data_t motor_data = {
            .value = 0,
            .flags = 0,
        };
        can_write_frame((uint8_t *)&motor_data, sizeof(motor_data_t));
    }
    break;
    case CMD_GET_FOC:
    {
        can_write_frame((uint8_t)cmd);
        foc_data_t foc_data = {
            .foc_1 = 0,
            .foc_2 = 0,
            .foc_3 = 0,
            .foc_en = 0,
        };
        can_write_frame((uint8_t *)&foc_data, sizeof(foc_data_t));
    }
    break;
    case CMD_ACK:
        can_write_frame((uint8_t)CMD_ACK);
        break;
    case CMD_FIRMWARE_UPDATE:
    {
        // TODO fw update via stream
        can_write_frame((uint8_t)cmd);
        error_data_t error_data = {
            .error_code = ERROR_NOT_IMPLEMENTED,
            .error_message = 0,
        };
        can_write_frame((uint8_t *)&error_data, sizeof(error_data_t));
    }
    break;
    case CMD_NETWORK_RESET:
    {
        // TODO reset network
        device_id_set(INVALID_DEVICE);
        operating_mode_set(OPERATING_MODE_DISCOVERY);
    }
    break;
    case CMD_ERASE_USER_STORE: // TODO
        break;
    case CMD_USER_STORE_UPDATE: // TODO
        break;
    case CMD_REVOKE_CONFIG:
    {
        device_id_set(INVALID_DEVICE);
        operating_mode_set(OPERATING_MODE_DISCOVERY);
    }
    break;
    // These cases are handled with a simple ACK
    case CMD_SET_POSITION:
    case CMD_SET_SERVO:
    case CMD_SET_LED:
    case CMD_SET_MOTOR:
    case CMD_SET_FOC:
    {
        can_write_frame((uint8_t)CMD_ACK);
    }
    break;
    default:
    {
        can_write_frame((uint8_t)CMD_ERROR);
        error_data_t error_data = {
            .error_code = ERROR_NOT_IMPLEMENTED,
            .error_message = 0,
        };
        can_write_frame((uint8_t *)&error_data, sizeof(error_data_t));
    }
    break;
    }
    can_end_frame(device_id_get() + DEVICE_BASE_ID);
}

static void discover(int16_t device_id)
{
    device_id_set(device_id);
    cached_id_set(device_id);
    operating_mode_set(OPERATING_MODE_CONTROL);
    can_set_device_filter(device_id_get() + DEVICE_BASE_ID);
    send_protocol_message(CMD_GET_INFO);
}

static void undiscover()
{
    device_id_set(INVALID_DEVICE);
    operating_mode_set(OPERATING_MODE_DISCOVERY);
    can_set_device_filter(DISCOVERY_ASSIGN_ID);
}

void handle_discovery(uint8_t *data, size_t length)
{
    if (!data || length < sizeof(discovery_data_t))
    {
        return;
    }
    discovery_data_t *msg = (discovery_data_t *)data;
    if (msg->serial_no == serial_no_get())
    {
        discover(msg->id_value);
    }
}

void handle_control(uint8_t *data, size_t length)
{
    if (!data)
    {
        return;
    }
    uint8_t is_controller = (data[0] & COMMAND_CONTROLLER_MASK);
    uint8_t command = (data[0] & COMMAND_CMD_MASK);
    switch (command)
    {
    case CMD_GET_INFO:
    {
        send_protocol_message(CMD_GET_INFO);
    }
    break;
    case CMD_GET_INFO_EXT:
    {
        send_protocol_message(CMD_GET_INFO_EXT);
    }
    break;
    case CMD_GET_POSITION:
    {
        send_protocol_message(CMD_GET_POSITION);
    }
    break;
    case CMD_SET_POSITION:
    {
        uint16_t position = data[1];
        position |= ((uint16_t)data[2]) << 8;
        motor_request_position(position);
        send_protocol_message(CMD_ACK);
    }
    break;
    case CMD_GET_STATUS:
    {
        send_protocol_message(CMD_GET_STATUS);
    }
    break;
    case CMD_GET_ANALOG:
    {
        send_protocol_message(CMD_GET_ANALOG);
    }
    break;
    case CMD_GET_SERVO:
    {
        send_protocol_message(CMD_GET_SERVO);
    }
    break;
    case CMD_SET_SERVO:
    {
        servo_data_t servo_data = {
            .srv0 = (uint16_t)(data[1]) | (((uint16_t)data[2]) << 8),
            .srv1 = (uint16_t)(data[3]) | (((uint16_t)data[4]) << 8),
            .update_mask = 0x3,
        };
        send_protocol_message(CMD_SET_SERVO);
    }
    break;
    case CMD_GET_LED:
    {
        send_protocol_message(CMD_GET_LED);
    }
    break;
    case CMD_SET_LED:
    {
        led_data_t led_data = {
            .sys = (uint8_t)(data[1]),
            .stat = (uint8_t)(data[2]),
            .update_mask = (uint8_t)(data[3]),
        };
        send_protocol_message(CMD_SET_LED);
    }
    break;
    case CMD_GET_MOTOR:
    {
        send_protocol_message(CMD_GET_MOTOR);
    }
    break;
    case CMD_SET_MOTOR:
    {
        // TODO override torque?
        uint8_t flags = data[1];
        digitalWrite(M_NRST, flags & MOTOR_RST_MASK);
        digitalWrite(M_NSLEEP, flags & MOTOR_SLEEP_MASK);
        send_protocol_message(CMD_ACK);
    }
    break;
    case CMD_GET_FOC:
    {
        send_protocol_message(CMD_GET_FOC);
    }
    break;
    case CMD_SET_FOC:
    {
        error_data_t error_data = {
            .error_code = ERROR_NOT_IMPLEMENTED,
            .error_message = 0,
        };
        send_error_message(&error_data);
    }
    break;
    case CMD_STREAM_START:
    {
        uint32_t stream_length = data[1] | (data[2] << 8);
        uint8_t checksum = data[3] | (data[4] << 8) | (data[5] << 16) | (data[6] << 24);
        handle_stream_start(stream_length, checksum);
    }
    break;
    case CMD_STREAM_READ:
    {
        // handle_stream_data(data + 1);
    }
    break;
    case CMD_STREAM_WRITE:
    {
        //
    }
    break;
    case CMD_ACK: // Noop
        break;
    case CMD_FIRMWARE_UPDATE:
    {
        // TODO fw update via stream
        error_data_t error_data = {
            .error_code = ERROR_NOT_IMPLEMENTED,
            .error_message = 0,
        };
        send_error_message(&error_data);
    }
    break;
    case CMD_NETWORK_RESET:
    {
        // Not valid on control channel
        // undiscover();
    }
    break;
    case CMD_ERASE_USER_STORE: // TODO
        break;
    case CMD_USER_STORE_UPDATE: // TODO
        break;
    case CMD_REVOKE_CONFIG:
    {
        undiscover();
    }
    break;
    default:
        break;
    }
}

void handle_broadcast(uint8_t *data, size_t length)
{
    if (!data || length < 1)
    {
        return;
    }
    uint8_t is_controller = (data[0] & COMMAND_CONTROLLER_MASK);
    uint8_t command = (data[0] & COMMAND_CMD_MASK);
    switch (command)
    {
    case CMD_NETWORK_RESET:
    {
        undiscover();
    }
    break;
    default:
        break;
    }
}

void handle_message(CAN_message_t can_msg)
{
    switch (can_msg.id)
    {
    case DISCOVERY_ASSIGN_ID:
    {
        if (operating_mode_get() == OPERATING_MODE_DISCOVERY)
        {
            handle_discovery(can_msg.buf, can_msg.len);
        }
    }
    break;
    case CONTROL_ID:
    default:
        if (operating_mode_get() == OPERATING_MODE_CONTROL)
        {

            if (can_msg.id >= DEVICE_BASE_ID)
            {
                uint16_t device_id = (can_msg.id - DEVICE_BASE_ID);
                if (device_id == device_id_get())
                {
                    handle_control(can_msg.buf, can_msg.len);
                }
            }
            else if (can_msg.id == CONTROL_ID)
            {
                handle_broadcast(can_msg.buf, can_msg.len);
            }
        }
        break;
    }
}

void send_discovery_query()
{
    can_start_frame();
    discovery_data_t discovery = {
        .serial_no = serial_no_get(),
        .id_value = cached_id_get(),
    };
    can_write_frame((uint8_t *)&discovery, sizeof(discovery_data_t));
    can_end_frame(DISCOVERY_REQUEST_ID);
}