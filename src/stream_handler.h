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

#include "parser.h"
#include "errors.h"
#include "flash.h"
#include "device.h"

class StreamHandler
{
    private:
        enum class State
        {
            IDLE,
            USER_STORE_STREAM,
            FIRMWARE_STREAM,
            FILE_STREAM,
        };
        State _state;
        uint32_t _stream_id;
        uint32_t _offset;
        uint32_t _length;
        uint32_t _chunk_id;
        uint8_t _flags;
    public:
        StreamHandler()
        {
            _state = State::IDLE;
            _stream_id = 0;
            _offset = 0;
            _length = 0;
            _chunk_id = 0;
        }

        void handle_stream_start(uint8_t stream_target, uint32_t stream_length, uint8_t flags, Parser* reply_to)
        {
            if (_state != State::IDLE)
            {
                // Busy
                if (reply_to)
                {
                    error_data_t error_data = {
                        .error_code = ERROR_BUSY,
                        .error_message = 0,
                    };
                    write_device_error(reply_to, &error_data);
                }
                return;
            }
            _flags = flags;
            _offset = 0;
            _length = stream_length;
            _chunk_id = 0;
            Flash* flash = Flash::get_instance();
            if (_flags & STREAM_FLAGS_WRITE)
            {
                switch(stream_target)
                {
                    case STREAM_TARGET_USER_STORE:
                    {
                        _state = State::USER_STORE_STREAM;
                        flash->erase();
                        write_ack(reply_to);
                    }
                        break;
                    case STREAM_TARGET_FIRMWARE:
                    {
                        /*
                        _state = State::FIRMWARE_STREAM;
                        _offset = 0;
                        _length = stream_length;
                        _chunk_id = 0;
                        reply.command = CMD_ACK;*/
                        error_data_t error_data = {
                            .error_code = ERROR_NOT_IMPLEMENTED,
                            .error_message = 0,
                        };
                        write_device_error(reply_to, &error_data);
                    }
                        break;
                    case STREAM_TARGET_FILE:
                    {
                        /*
                        _state = State::FILE_STREAM;
                        _offset = 0;
                        _length = stream_length;
                        _chunk_id = 0;
                        reply.command = CMD_ACK;
                        */
                        error_data_t error_data = {
                            .error_code = ERROR_NOT_IMPLEMENTED,
                            .error_message = 0,
                        };
                        write_device_error(reply_to, &error_data);
                    }
                        break;
                    default:
                    {                        error_data_t error_data = {
                            .error_code = ERROR_INVALID_ARGUMENT,
                            .error_message = 0,
                        };
                        write_device_error(reply_to, &error_data);
                    }
                        break;
                }
            } else {
                write_ack(reply_to);
            }
        }

        void handle_stream_data(uint8_t sequence_id, uint8_t* buffer, size_t length, Parser* reply_to)
        {
            Flash* flash = Flash::get_instance();
            uint32_t flash_buffer = 0xFFFFFFFF;
            if (_flags & STREAM_FLAGS_WRITE)
            {
                if (sequence_id == _chunk_id)
                {
                    switch (_state)
                    {
                        case State::FIRMWARE_STREAM:
                        {
                            error_data_t error_data = {
                                .error_code = ERROR_NOT_IMPLEMENTED,
                                .error_message = 0,
                            };
                            write_device_error(reply_to, &error_data);
                            break;
                        }
                        case State::USER_STORE_STREAM:
                            // Write flash and increment offset
                            // If less than 32 bytes left, pad with 0xFF
                            memcpy(&flash_buffer, buffer, length);
                            if (flash->write(_offset, &flash_buffer, 1))
                            {
                                _offset += sizeof(flash_buffer);
                                ++_chunk_id;
                                write_ack(reply_to);
                            } else {
                                error_data_t error_data = {
                                    .error_code = ERROR_WRITE_FAILED,
                                    .error_message = 0,
                                };
                                write_device_error(reply_to, &error_data);
                            }
                            if (_offset >= _length)
                            {
                                _state = State::IDLE;
                            }
                            break;
                        case State::IDLE:
                            break;
                        default:
                            _state = State::IDLE;
                            break;
                    }
                } else {
                    error_data_t error_data = {
                        .error_code = ERROR_INVALID_SEQUENCE,
                        .error_message = sequence_id,
                    };
                    write_device_error(reply_to, &error_data);
                }
            } else {
                // Read, check sequence id
                if (sequence_id != _chunk_id)
                {
                    // Re-align stream
                    _offset -= (_chunk_id - sequence_id) * 4;
                    _chunk_id = sequence_id;
                }
                switch(_state)
                {
                    case State::USER_STORE_STREAM:
                        // Read flash and increment offset
                        // If less than 32 bytes left, pad with 0xFF

                        if (flash->read(_offset, &flash_buffer, 1))
                        {
                            _offset += sizeof(flash_buffer);
                            stream_data_t stream_data;
                            stream_data.sequence_id = _chunk_id;
                            ++_chunk_id;
                            memcpy(stream_data.data, &flash_buffer, sizeof(flash_buffer));
                            write_device_stream_data(reply_to, &stream_data);
                        }
                        if (_offset >= _length)
                        {
                            _state = State::IDLE;
                            write_ack(reply_to);
                        }
                        break;
                    default:
                    {
                        error_data_t error_data = {
                            .error_code = ERROR_UNKNOWN_STATE,
                            .error_message = 0,
                        };
                        write_device_error(reply_to, &error_data);
                        _state = State::IDLE;
                    }
                        break;
                }
            }

        }
};

#endif // _STREAM_HANDLER_H_
