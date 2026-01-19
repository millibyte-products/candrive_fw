#ifndef _CAN_PARSER_H_
#define _CAN_PARSER_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <STM32_CAN.h>
#include "protocol.h"
#include "parser.h"
#include "command_handler.h"

#define CAN_FRAME_MAX_DATA_LENGTH (8)

class CANParser : public Parser
{
private:
    STM32_CAN instance;

    void set_filter(uint8_t message_channel)
    {
        // Always listen to control messages on channel 0
        instance.setFilterDualID(0, CONTROL_ID, message_channel, STD, STD);
        // Dynamically set between discovery and device message channels
        instance.setFilter(0, true);
    }
public:
    CANParser(dev_handler_t dev, dis_handler_t dis, bro_handler_t bro, CAN_TypeDef* can = CAN1) : Parser(dev, dis, bro), instance(can)
    {
        instance.setBaudRate(1000000);
        instance.begin();
        // Filter for controller messages
        set_filter(DISCOVERY_ASSIGN_ID);
    }

    void set_device_filter(int16_t device_id) override
    {
        set_filter((uint8_t)device_id);
    }

    void read() override
    {
        CAN_message_t can_msg;
        if (instance.read(can_msg))
        {
            switch (can_msg.id)
            {
                case DISCOVERY_ASSIGN_ID:
                    {
                        discovery_message_t dmsg;
                        memcpy(&dmsg, can_msg.buf, can_msg.len);
                        if (dis_handler(&dmsg, this) == PARSER_OK)
                        {
                            // Start listening to device messages on our channel
                            set_filter(get_device_id() + DEVICE_BASE_ID);
                        }
                    }
                break;
                case CONTROL_ID:
                default:
                    if (can_msg.id >= DEVICE_BASE_ID)
                    {
                        uint8_t device_id = (can_msg.id - DEVICE_BASE_ID);
                        device_message_t dmsg;
                        memcpy(&dmsg, can_msg.buf, can_msg.len);
                        dev_handler(device_id, &dmsg, this);
                    } else if (can_msg.id == CONTROL_ID) {
                        // Handle control messages
                        device_message_t dmsg;
                        memcpy(&dmsg, can_msg.buf, can_msg.len);
                        bro_handler(&dmsg, this);
                    }
                    break;
            }
        }
    }

    void write(uint32_t id, uint8_t* buffer, size_t length) override
    {
        if (buffer && length <= CAN_FRAME_MAX_DATA_LENGTH)
        {
            CAN_message_t msg;
            msg.id = id;
            msg.len = length;
            memcpy(msg.buf, buffer, length);
            instance.write(msg);
        }
    }

    void end_frame(uint32_t id) override
    {
        write(id, frame_buffer, frame_length());
    }
};

#endif // _CAN_PARSER_H_
