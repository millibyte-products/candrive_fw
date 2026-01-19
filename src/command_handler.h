#ifndef _COMMAND_HANDLER_H_
#define _COMMAND_HANDLER_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#define _TASK_STD_FUNCTION
#define _TASK_SCHEDULING_OPTIONS

#include <unordered_map>

#include <TaskScheduler.h>

#include <Arduino.h>

class CommandHandler;

#include "device.h"
#include "parser.h"
#include "protocol.h"
#include "serial_parser.h"
#include "can_parser.h"
#include "errors.h"
#include "stream_handler.h"
#include "motor.h"

#define LED_DUTY_DEFAULT (25)
#define FOC_PWM_FREQ (50000)

class CommandHandler
{
private:
    HardwareTimer led_timer;
    HardwareTimer servo_timer;
    Motor motor;
    SerialParser serial_interface;
    CANParser can_interface;
    Scheduler ts;
    Task critical_task;
    Task standard_task;
    Task lazy_task;
    Task discovery_task;
    bool sys_state;
    bool stat_state;
    int16_t previous_device_id;
    StreamHandler stream_handler;
    std::unordered_map<int, uint16_t> servo_values;

    void set_pwm(HardwareTimer* timer, int pin, uint16_t duty, uint32_t freq)
    {
        if (timer)
        {

            uint32_t computed_duty = ((uint32_t)duty * freq) / 65535;
            uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
            timer->setPWM(channel, pin, freq, computed_duty);
        }
    }

    void set_led(int pin, uint32_t duty) {
        set_pwm(&led_timer, pin, duty, LED_FREQ);
    }

    void set_servo(int pin, uint16_t duty) {
        servo_values[pin] = duty;
        set_pwm(&servo_timer, pin, duty, SERVO_FREQ);
    }

    void get_servo(int pin, uint16_t* duty) {
        std::unordered_map<int, uint16_t>::iterator it = servo_values.find(pin);
        if (it != servo_values.end()) {
            *duty = it->second;
        } else {
            *duty = 0;
        }
    }

    void handle_endstop0(void)
    {
        // Overwrite position with maximum value
        // TODO
    }

    void handle_endstop1(void)
    {
        // Overwrite position with minimum value
        // TODO
    }

    void handle_motor_fault(void)
    {
        // De-power motor, blink sys led
    }

    void update_critical()
    {
        motor.update();
    }

    void update_standard()
    {
        serial_interface.read();
        can_interface.read();
    }

    void update_lazy()
    {
        sys_state = !sys_state;
        set_led(LED_SYS_PWM, sys_state ? LED_DUTY_DEFAULT : 0);
        set_led(LED_STAT_PWM, stat_state ? LED_DUTY_DEFAULT : 0);
    }

    void update_discovery()
    {
        discovery_message_t discovery_message {
            .serial_no = get_serial_no(),
            .id_value = static_cast<uint8_t>(previous_device_id),
        };
        can_interface.start_frame();
        can_interface.write_frame((uint8_t*)&discovery_message, sizeof(discovery_message_t));
        can_interface.end_frame(DISCOVERY_REQUEST_ID);
    }

    void set_discovery_mode(bool enable, Parser* parser_instance)
    {
        if (enable)
        {
            discovery_task.enable();
            set_device_id(INVALID_DEVICE);
            if (parser_instance)
            {
                parser_instance->set_device_filter(DISCOVERY_ASSIGN_ID);
            }
        } else {
            discovery_task.disable();
            previous_device_id = get_device_id();
            if (parser_instance)
            {
                parser_instance->set_device_filter(get_device_id() + DEVICE_BASE_ID);
            }
        }
    }

    int32_t handle_discovery_message(discovery_message_t* msg, Parser* reply_to)
    {
        if (msg && msg->serial_no == get_serial_no())
        {
            set_device_id((int16_t)msg->id_value);
            set_discovery_mode(false, reply_to);
            if (reply_to)
            {
                write_device_info(reply_to);
            }
        }
        return PARSER_OK;
    }

    int32_t handle_device_message(uint8_t device_id, device_message_t *msg, Parser* reply_to)
    {
        if ((int16_t)device_id == get_device_id() && msg && (msg->command & COMMAND_CONTROLLER_MASK))
        {
            uint8_t command = msg->command & COMMAND_CMD_MASK;
            switch(command)
            {
                case CMD_GET_INFO:
                    {
                        write_device_info(reply_to);
                    }
                    break;
                case CMD_GET_INFO_EXT:
                    {
                        write_device_info_ext(reply_to);
                    }
                    break;
                case CMD_SET_POSITION:
                    {
                        uint16_t position = msg->data[1] | (msg->data[2] << 8);
                        motor.request_angle(position);
                        write_ack(reply_to);
                    }
                    break;
                case CMD_GET_POSITION:
                    {
                        write_device_position(reply_to, (uint16_t)motor.get_angle() );
                    }
                    break;
                case CMD_GET_STATUS:
                    {
                        uint8_t flags =
                                (0 ? digitalRead(ENDSTOP0) == LOW : 1) |
                                (0 ? digitalRead(ENDSTOP1) == LOW : 1) << 1 |
                                (0 ? digitalRead(MISC) == LOW : 1) << 2 |
                                (0 ? digitalRead(M_NFAULT) == LOW : 1) << 3 |
                                0 << 4;
                        write_device_status(reply_to, flags);
                    }
                    break;
                case CMD_GET_ANALOG:
                    {
                        device_analog_t analog_data = {
                            .a0 = static_cast<uint16_t>(analogRead(A0)),
                            .a1 = static_cast<uint16_t>(analogRead(A1)),
                        };
                        write_device_analog(reply_to, &analog_data);
                    }
                    break;
                case CMD_SET_SERVO:
                    {
                        uint16_t srv0 = msg->data[1] | (msg->data[2] << 8);
                        uint16_t srv1 = msg->data[3] | (msg->data[4] << 8);
                        set_servo(SRV0_PWM, srv0);
                        set_servo(SRV1_PWM, srv1);
                        write_ack(reply_to);
                    }
                    break;
                case CMD_GET_SERVO:
                    {

                        device_servo_t servo_data;
                        get_servo(SRV0_PWM, &servo_data.srv0);
                        get_servo(SRV1_PWM, &servo_data.srv1);
                        write_device_servo(reply_to, &servo_data);
                    }
                    break;
                case CMD_SET_LED:
                    {
                        lazy_task.disable();
                        uint8_t sys = msg->data[1];
                        uint8_t stat = msg->data[2];
                        uint8_t update_mask = msg->data[3];
                        if (update_mask & LED_STAT)
                        {
                            set_led(LED_STAT_PWM, stat);
                        }
                        if (update_mask & LED_SYS)
                        {
                            set_led(LED_SYS_PWM, sys);
                        }
                        write_ack(reply_to);
                    }
                    break;
                case CMD_SET_MOTOR:
                    {
                        // TODO override torque?
                        uint8_t flags = msg->data[1];
                        digitalWrite(M_NRST, flags & MOTOR_RST_MASK);
                        digitalWrite(M_NSLEEP, flags & MOTOR_SLEEP_MASK);
                        write_ack(reply_to);
                    }
                    break;
                case CMD_GET_MOTOR:
                    {
                        device_motor_t motor_data = {
                            .value = 0,
                            .flags = 0,
                        };
                        write_device_motor(reply_to, &motor_data);
                    }
                    break;
                case CMD_SET_FOC:
                    {
                        // TODO direct foc/pwm control
                        error_data_t error_data = {
                            .error_code = ERROR_NOT_IMPLEMENTED,
                            .error_message = 0,
                        };
                        write_device_error(reply_to, &error_data);
                    }
                    break;
                case CMD_STREAM_START:
                    {
                        uint8_t stream_target = msg->data[1];
                        uint32_t stream_length = msg->data[2] | (msg->data[3] << 8) | (msg->data[4] << 16) | (msg->data[5] << 24);
                        uint8_t flags = msg->data[6];
                        stream_handler.handle_stream_start(stream_target, stream_length, flags, reply_to);
                    }
                    break;
                case CMD_STREAM_DATA:
                    {
                        uint8_t sequence_id = msg->data[1];
                        uint8_t data[4] = {
                            msg->data[2],
                            msg->data[3],
                            msg->data[4],
                            msg->data[5],
                        };
                        stream_handler.handle_stream_data(sequence_id, data, 4, reply_to);
                    }
                    break;
                case CMD_ACK:
                    {
                        // No need to respond
                    }
                    break;
                case CMD_START_FW_UPDATE:
                    {
                        // TODO fw update via stream
                        error_data_t error_data = {
                            .error_code = ERROR_NOT_IMPLEMENTED,
                            .error_message = 0,
                        };
                        write_device_error(reply_to, &error_data);
                    }
                    break;
                case CMD_NETWORK_RESET:
                    {
                        // TODO reset network
                        set_device_id(INVALID_DEVICE);
                        set_discovery_mode(true, reply_to);
                    }
                    break;
                case CMD_ERASE_USER_STORE:
                    break;
                case CMD_OVERWRITE_USER_STORE:
                    break;
                case CMD_REVOKE_CONFIG:
                    {
                        set_device_id(INVALID_DEVICE);
                        set_discovery_mode(true, reply_to);
                    }
                    break;
                default:
                    return PARSER_ERROR;
                    break;
            }
        }
        return PARSER_OK;
    }

    int32_t handle_broadcast_message(device_message_t* msg, Parser* reply_to)
    {
        switch (msg->command)
        {
            case CMD_NETWORK_RESET:
            {
                set_device_id(INVALID_DEVICE);
                set_discovery_mode(true, reply_to); // TODO: reply_to is not used
            }
        }
        return PARSER_OK;
    }

public:
    CommandHandler() : led_timer(TIM3),
        servo_timer(TIM4),
        serial_interface(std::bind(&CommandHandler::handle_device_message,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3),
                         std::bind(&CommandHandler::handle_discovery_message,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2),
                        std::bind(&CommandHandler::handle_broadcast_message,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2)),
        can_interface(std::bind(&CommandHandler::handle_device_message,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2,
                                std::placeholders::_3),
                      std::bind(&CommandHandler::handle_discovery_message,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2),
                      std::bind(&CommandHandler::handle_broadcast_message,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2)),
        ts(),
        critical_task(3 * TASK_MILLISECOND, -1, std::bind(&CommandHandler::update_critical, this), &ts, true),
        standard_task(10 * TASK_MILLISECOND, -1, std::bind(&CommandHandler::update_standard, this), &ts, true),
        lazy_task(250 * TASK_MILLISECOND, -1, std::bind(&CommandHandler::update_lazy, this), &ts, true),
        discovery_task(20000 * TASK_MILLISECOND, -1, std::bind(&CommandHandler::update_discovery, this), &ts, true),
        sys_state(false),
        stat_state(false),
        previous_device_id(0) // TOOD: load from non-volatile storage
    {
        // put your setup code here, to run once:
        pinMode(ENDSTOP0, INPUT_PULLUP);
        pinMode(ENDSTOP1, INPUT_PULLUP);
        pinMode(A0, INPUT_ANALOG);
        pinMode(A1, INPUT_ANALOG);
        pinMode(M_NRST, OUTPUT);
        pinMode(M_NSLEEP, OUTPUT);
        pinMode(M_NFAULT, INPUT_PULLUP);
        // STM32 specific PWM setup

        set_led(LED_SYS_PWM, 25);
        set_led(LED_STAT_PWM, 0);

        set_servo(SRV0_PWM, 0);
        set_servo(SRV1_PWM, 0);

        attachInterrupt(digitalPinToInterrupt(ENDSTOP0), std::bind(&CommandHandler::handle_endstop0, this), FALLING);
        attachInterrupt(digitalPinToInterrupt(ENDSTOP1), std::bind(&CommandHandler::handle_endstop1, this), FALLING);
        attachInterrupt(digitalPinToInterrupt(M_NFAULT), std::bind(&CommandHandler::handle_motor_fault, this), FALLING);

        motor.init();
        critical_task.setSchedulingOption(TASK_SCHEDULE_NC);
        lazy_task.setSchedulingOption(TASK_INTERVAL);
        discovery_task.setSchedulingOption(TASK_INTERVAL);
        standard_task.setSchedulingOption(TASK_SCHEDULE_NC);
        critical_task.enable();
        standard_task.enable();
        lazy_task.enable();

        sys_state = true;
        stat_state = false;

        set_discovery_mode(true, nullptr);
    }

    void run()
    {
        ts.execute();
        yield();
    }
};

#endif // _COMM_H_
