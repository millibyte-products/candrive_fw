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
#include "parser.h"
#include "command_handler.h"

class CANParser : public Parser
{
private:
    STM32_CAN instance;
public:
    CANParser(dev_handler_t dev, dis_handler_t dis, CAN_TypeDef*can = CAN1) : Parser(dev, dis), instance(can)
    {
        instance.setBaudRate(1000000);
        instance.begin();
        // Filter for controller messages
        instance.setFilterDualID(0, CONTROLLER_ID, CONTROLLER_DISCOVERY, STD, STD);
    }

    void read() override
    {
        CAN_message_t can_msg;
        if (instance.read(can_msg))
        {
            switch (can_msg.id)
            {
                case CONTROLLER_ID:
                {
                    device_message_t dmsg;
                    memcpy(&dmsg, can_msg.buf, can_msg.len);
                    dev_handler(&dmsg, this);
                }
                    break;
                case CONTROLLER_DISCOVERY:
                {
                    discovery_message_t dmsg;
                    if (can_msg.len != sizeof(discovery_message_t))
                    {
                        // Abort, discarding message
                        return;
                    }
                    memcpy(&dmsg, can_msg.buf, sizeof(discovery_message_t));
                    dis_handler(&dmsg, this);
                }
                    break;
                default:
                    break;
            }
        }
    }

    void write(uint32_t id, uint8_t* buffer, size_t length) override
    {
        if (buffer && length <= 8)
        {
            CAN_message_t msg;
            msg.id = id;
            msg.len = length;
            memcpy(msg.buf, buffer, length);
            instance.write(msg);
        }
    }
};

#endif // _CAN_PARSER_H_
