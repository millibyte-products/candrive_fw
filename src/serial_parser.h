#ifndef _SERIAL_PARSER_H_
#define _SERIAL_PARSER_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <HardwareSerial.h>
#include "parser.h"
#include "command_handler.h"

//HardwareSerial Serial1(PA10, PA9); 
//HardwareSerial Serial2(PA3, PA2);
HardwareSerial Serial3(PB11, PB10);

class SerialParser : public Parser
{
private:
    HardwareSerial* instance;
public:
    SerialParser(dev_handler_t dev, dis_handler_t dis, HardwareSerial *ser = &Serial3) : Parser(dev, dis), instance(ser)
    {
        if (instance)
        {
            //instance->setRx(PB11);
            //instance->setTx(PB10);
            instance->begin(115200);
        }
    }

    void read() override
    {
        if (instance->available())
        {
            uint8_t length = instance->read();
            if (length > sizeof(device_message_t))
            {
                instance->flush();
                return;
            }
            device_message_t dmsg;
            instance->readBytes((uint8_t*)&dmsg, length);
            dev_handler(&dmsg, this);
        }
    }

    void write(uint32_t id, uint8_t* buffer, size_t length) override
    {
        (void)id;
        if (buffer)
        {
            instance->write(buffer, length);
        }
    }
};

#endif // _SERIAL_PARSER_H_
