#ifndef _PARSER_H_
#define _PARSER_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <stddef.h>
#include <functional>
#include "protocol.h"

class Parser;

#define PARSER_OK (0)
#define PARSER_ERROR (-1)

typedef std::function<int32_t(uint8_t device_id, device_message_t*, Parser*)> dev_handler_t;
typedef std::function<int32_t(discovery_message_t*,Parser*)> dis_handler_t;
typedef std::function<int32_t(device_message_t*,Parser*)> bro_handler_t;

class Parser
{
protected:
     dev_handler_t dev_handler;
     dis_handler_t dis_handler;
     bro_handler_t bro_handler;
     uint8_t frame_buffer[11];
     uint8_t *frame_ptr;
public:
    Parser(dev_handler_t dev, dis_handler_t dis, bro_handler_t bro): dev_handler(dev), dis_handler(dis), bro_handler(bro)
    {}
    virtual void set_device_filter(int16_t device_id) = 0;
    virtual void read() = 0;
    virtual void write(uint32_t id, uint8_t* buffer, size_t length) = 0;
    virtual void start_frame()
    {
        frame_ptr = frame_buffer;
    }
    virtual void write_frame(uint8_t* buffer, size_t length)
    {
        if (frame_ptr + length > frame_buffer + sizeof(frame_buffer))
        {
            return;
        }
        memcpy(frame_ptr, buffer, length);
        frame_ptr += length;
    };
    virtual void write_frame(uint8_t data)
    {
        if (frame_ptr + sizeof(data) > frame_buffer + sizeof(frame_buffer))
        {
            return;
        }
        *frame_ptr++ = data;
    };
    virtual void write_frame(uint16_t data)
    {
        if (frame_ptr + sizeof(data) > frame_buffer + sizeof(frame_buffer))
        {
            return;
        }
        *frame_ptr++ = data >> 8;
        *frame_ptr++ = data & 0xFF;
    };
    virtual void write_frame(uint32_t data)
    {
        if (frame_ptr + sizeof(data) > frame_buffer + sizeof(frame_buffer))
        {
            return;
        }
        *frame_ptr++ = data >> 24;
        *frame_ptr++ = (data >> 16) & 0xFF;
        *frame_ptr++ = (data >> 8) & 0xFF;
        *frame_ptr++ = data & 0xFF;
    };
    virtual uint8_t frame_length()
    {
        return (uint8_t)(frame_ptr - frame_buffer);
    }
    virtual void end_frame(uint32_t id) = 0;
};


inline void write_device_info(Parser* parser)
{
    if (parser)
    {
        device_info_t info = {
            .serial_no = get_serial_no(),
            .fw_ver_major = get_fw_major(),
            .fw_ver_minor = get_fw_minor(),
            .fw_ver_patch = get_fw_patch(),
        };
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_INFO);
        parser->write_frame((uint8_t*)&info, sizeof(device_info_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}

inline void write_device_info_ext(Parser* parser)
{
    if (parser)
    {
        device_info_ext_t info = {
            .flags = get_last_reset_reason() & RESET_REASON_MASK | (0 << 4), // No fault flags yet
            .temperature = 0,
        };
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_INFO_EXT);
        parser->write_frame((uint8_t*)&info, sizeof(device_info_ext_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_position(Parser* parser, uint16_t pos)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_POSITION);
        parser->write_frame(pos);
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_status(Parser* parser, uint8_t flags)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_STATUS);
        parser->write_frame(flags);
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_analog(Parser* parser, device_analog_t* analog)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_ANALOG);
        parser->write_frame((uint8_t*)analog, sizeof(device_analog_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_servo(Parser* parser, device_servo_t* servo)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_SERVO);
        parser->write_frame((uint8_t*)servo, sizeof(device_servo_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_led(Parser* parser, device_led_t* led)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_LED);
        parser->write_frame((uint8_t*)led, sizeof(device_led_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_motor(Parser* parser, device_motor_t* motor)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_MOTOR);
        parser->write_frame((uint8_t*)motor, sizeof(device_motor_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_foc(Parser* parser, device_foc_t* foc)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_GET_FOC);
        parser->write_frame((uint8_t*)foc, sizeof(device_foc_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_stream_start(Parser* parser, stream_start_t* stream_start)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_STREAM_START);
        parser->write_frame((uint8_t*)stream_start, sizeof(stream_start_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_stream_data(Parser* parser, stream_data_t* stream_data)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_STREAM_DATA);
        parser->write_frame((uint8_t*)stream_data, sizeof(stream_data_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}
inline void write_device_error(Parser* parser, error_data_t* error)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_ERROR);
        parser->write_frame((uint8_t*)error, sizeof(error_data_t));
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}

inline void write_ack(Parser* parser)
{
    if (parser)
    {
        parser->start_frame();
        parser->write_frame((uint8_t)CMD_ACK);
        parser->end_frame(get_device_id() + DEVICE_BASE_ID);
    }
}

#endif // _PARSER_H_
