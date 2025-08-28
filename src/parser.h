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

typedef std::function<void(device_message_t*,Parser*)> dev_handler_t;
typedef std::function<void(discovery_message_t*,Parser*)> dis_handler_t;

class Parser
{
protected:
     dev_handler_t dev_handler;
     dis_handler_t dis_handler;
public:
    Parser(dev_handler_t dev, dis_handler_t dis) : dev_handler(dev), dis_handler(dis)
    {}
    virtual void read() = 0;
    virtual void write(uint32_t id, uint8_t* buffer, size_t length) = 0;
};

#endif // _PARSER_H_
