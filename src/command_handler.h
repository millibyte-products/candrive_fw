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

#include <TaskScheduler.h>

#include <Arduino.h>
#include <BLDCMotor.h>
#include <drivers/BLDCDriver3PWM.h>
#include <encoders/MT6701/MagneticSensorMT6701SSI.h>

class CommandHandler;

#include "device.h"
#include "parser.h"
#include "protocol.h"
#include "kinematics.h"
#include "serial_parser.h"
#include "can_parser.h"

#define LED_DUTY_DEFAULT (25)

class CommandHandler
{
private:
    HardwareTimer led_timer;
    HardwareTimer servo_timer;
    MagneticSensorMT6701SSI encoder;
    BLDCMotor motor;
    BLDCDriver3PWM motor_driver;
    KinematicsModel kinematics;
    SerialParser serial_interface;
    CANParser can_interface;
    Scheduler ts;
    Task iut;
    Task ims;
    Task ldt;
    bool sys_state;
    bool stat_state;

    inline void set_pwm(HardwareTimer* timer, int pin, uint32_t duty, uint32_t freq)
    {
        if (timer)
        {
            PinName name = digitalPinToPinName(pin);
            uint32_t channel = STM_PIN_CHANNEL(name);
            timer->setPWM(channel, pin, freq, duty);
        }
    }

    void set_led(int pin, uint32_t duty) {
        set_pwm(&led_timer, pin, duty, LED_FREQ);
    }

    void set_servo(int pin, uint32_t duty) {
        set_pwm(&servo_timer, pin, duty, SERVO_FREQ);
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

    void update_motion_system()
    {
        motor.loopFOC();
        encoder.update();
        float angle = encoder.getAngle();
        float target_voltage = kinematics.calculate_voltage(angle);
        motor.move(target_voltage);
    }

    void update_external_interfaces()
    {
        serial_interface.read();
        can_interface.read();
    }

    void update_leds()
    {
        sys_state = !sys_state;
        set_led(LED_SYS_PWM, sys_state ? LED_DUTY_DEFAULT : 0);
        set_led(LED_STAT_PWM, stat_state ? LED_DUTY_DEFAULT : 0);
    }
    
    void handle_discovery_message(discovery_message_t* msg, Parser* reply_to)
    {
        if (msg && msg->serial_no == get_serial_no())
        {
            set_device_id(msg->device_id);
            uint8_t buffer[2] = {
                msg->device_id,
                DEVICE_ACK
            };
            if (reply_to)
            {
                reply_to->write(DEVICE_DISCOVERY, buffer, 2);
            }
        }
    }

    void handle_device_message(device_message_t *msg, Parser* reply_to)
    {
        if (msg && msg->device_id == get_device_id())
        {
            switch(msg->cmd)
            {
                case CONTROLLER_GET_INFO:
                {
                    device_message_t reply {
                        .device_id = get_device_id(),
                        .cmd = DEVICE_INFO,
                        .info_data = {
                            .serial_no = get_serial_no(),
                            .fw_ver = get_fw_ver(),
                            .hw_ver = get_hw_ver(),
                        },
                    };
                    if (reply_to)
                    {
                        reply_to->write(DEVICE_GENERAL, (uint8_t*)&reply, DEVICE_HEADER_SIZE + sizeof(device_info_t));
                    }
                }
                break;
                case CONTROLLER_SET_POSITION:
                {
                    kinematics.set_position_req(((float)msg->position_data.position) / 10000);
                    device_message_t reply
                    {
                        .device_id = get_device_id(),
                        .cmd = DEVICE_ACK,
                    };
                    if (reply_to)
                    {
                        reply_to->write(DEVICE_GENERAL, (uint8_t*)&reply, DEVICE_HEADER_SIZE);
                    }
                }
                break;
                case CONTROLLER_GET_STATUS:
                {
                    device_message_t reply {
                        .device_id = get_device_id(),
                        .cmd = DEVICE_STATUS,
                        .status_data = {
                            .endstop0 = digitalRead(ENDSTOP0),
                            .endstop1 = digitalRead(ENDSTOP1),
                            .misc = digitalRead(MISC),
                            .fault = digitalRead(M_NFAULT),
                            .mag = 0, // we can't read this yet
                        },
                    };
                    if (reply_to)
                    {
                        reply_to->write(DEVICE_GENERAL, (uint8_t*)&reply, DEVICE_HEADER_SIZE + sizeof(device_status_t));
                    }
                }
                break;
                case CONTROLLER_GET_ANALOG:
                {
                    device_message_t reply {
                        .device_id = get_device_id(),
                        .cmd = DEVICE_ANALOG,
                        .analog_data = {
                            .a0 = analogRead(A0),
                            .a1 = analogRead(A1),
                        },
                    };
                    if (reply_to)
                    {
                        reply_to->write(DEVICE_GENERAL, (uint8_t*)&reply, DEVICE_HEADER_SIZE + sizeof(device_analog_t));
                    }
                }
                break;
                case CONTROLLER_SET_SERVO:
                {
                    set_servo(SRV0_PWM, msg->servo_data.srv0);
                    set_servo(SRV1_PWM, msg->servo_data.srv1);
                    device_message_t reply {
                        .device_id = get_device_id(),
                        .cmd = DEVICE_ACK,
                    };
                    if (reply_to)
                    {
                        reply_to->write(DEVICE_GENERAL, (uint8_t*)&reply, DEVICE_HEADER_SIZE);
                    }
                }
                break;
                case CONTROLLER_SET_LED:
                {
                    ldt.disable();
                    set_led(LED_SYS_PWM, msg->led_data.sys);
                    set_led(LED_STAT_PWM, msg->led_data.stat);
                    device_message_t reply {
                        .device_id = get_device_id(),
                        .cmd = DEVICE_ACK,
                    };
                    if (reply_to)
                    {
                        reply_to->write(DEVICE_GENERAL, (uint8_t*)&reply, DEVICE_HEADER_SIZE);
                    }
                }
                break;
                case CONTROLLER_SET_MOTOR:
                {
                    // TODO override torque?
                    digitalWrite(M_NRST, msg->motor_data.rst);
                    digitalWrite(M_NSLEEP, msg->motor_data.sleep);
                    device_message_t reply {
                        .device_id = get_device_id(),
                        .cmd = DEVICE_ACK,
                    };
                    if (reply_to)
                    {
                        reply_to->write(DEVICE_GENERAL, (uint8_t*)&reply, DEVICE_HEADER_SIZE);
                    }
                }
                break;
                case CONTROLLER_SET_FOC:
                {
                    // TODO direct foc/pwm control
                }
                break;
                case CONTROLLER_STREAM_START:
                {
                    // TODO direct stream
                }
                break;
                case CONTROLLER_STREAM_DATA:
                {
                    // TODO direct stream
                }
                break;
                case CONTROLLER_ACK:
                {
                    // No need to respond
                }
                break;
                case CONTROLLER_START_FW_UPDATE:
                {
                    // TODO fw update via stream
                }
                break;
                default:
                    break;
            }
        }
    }

public:
    CommandHandler() : 
        encoder(ENCODER_CSN),
        motor(11, 11.1),
        motor_driver(FOC_IN1_PWM, FOC_IN2_PWM, FOC_IN3_PWM, FOC_EN),
        led_timer(TIM3), 
        servo_timer(TIM4),
        iut(10 * TASK_MILLISECOND, -1, std::bind(&CommandHandler::update_external_interfaces, this), &ts, true),
        ims(3 * TASK_MILLISECOND, -1, std::bind(&CommandHandler::update_motion_system, this), &ts, true),
        ldt(250 * TASK_MILLISECOND, -1, std::bind(&CommandHandler::update_leds, this), &ts, true),
        serial_interface(std::bind(&CommandHandler::handle_device_message,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2),
                         std::bind(&CommandHandler::handle_discovery_message,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2)),
        can_interface(std::bind(&CommandHandler::handle_device_message,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2),
                      std::bind(&CommandHandler::handle_discovery_message,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2))
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

        attachInterrupt(ENDSTOP0, std::bind(&CommandHandler::handle_endstop0, this), FALLING);
        attachInterrupt(ENDSTOP1, std::bind(&CommandHandler::handle_endstop1, this), FALLING);
        attachInterrupt(M_NFAULT, std::bind(&CommandHandler::handle_motor_fault, this), FALLING);

        // Enable motor
        digitalWrite(M_NSLEEP, HIGH);
        // Disable motor sleep 
        digitalWrite(M_NRST, HIGH);
        SPI.setMOSI(PA7);
        SPI.setMISO(PA6);
        SPI.setSCLK(PA5);
        SPI.begin();
        encoder.init(&SPI);
        motor_driver.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
        motor_driver.voltage_limit = MOTOR_OPERATING_VOLTAGE;
        motor_driver.init();
        motor.current_limit = MOTOR_CURRENT_LIMIT;
        motor.torque_controller = TorqueControlType::voltage;
        motor.controller = MotionControlType::torque;
        motor.linkSensor(&encoder);
        motor.linkDriver(&motor_driver);
        motor.init();
        motor.useMonitoring(Serial3);
        motor.voltage_sensor_align = MOTOR_OPERATING_VOLTAGE / 2.0f;
        motor.initFOC();
        encoder.update();
        kinematics.set_zero(encoder.getSensorAngle());
        iut.setSchedulingOption(TASK_SCHEDULE_NC);
        ldt.setSchedulingOption(TASK_INTERVAL);
        ims.setSchedulingOption(TASK_SCHEDULE_NC);
        sys_state = true;
        stat_state = false;
    }

    void run()
    {
        ts.execute();
    }
};

#endif // _COMM_H_
