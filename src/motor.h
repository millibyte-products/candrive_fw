#ifndef _MOTOR_H_
#define _MOTOR_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <stdint.h>
#include <drivers/BLDCDriver3PWM.h>
#include <drivers/hardware_specific/stm32/stm32_mcu.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <BLDCMotor.h>

#define FOC_PWM_FREQ (50000)
static const float PRECISION_16 = 65536.0f;

class Motor
{
private:
    float voltage_req;
    float current_req;
    float rpm_req;
    float velocity_req;
    float position_req;
    float angle_req;
    float torque_req;
    float zero_angle;
    MagneticSensorMT6701SSI encoder;
    BLDCMotor motor;
    BLDCDriver3PWM motor_driver;
public:
    // SPI should be intiialized before calling init()
    Motor(float voltage = 12.0f, float current = 0.0f, float rpm = 0.0f, float velocity = 0.0f, float position = 0.0f, float torque = 0.0f) :
        encoder(ENCODER_CSN),
        motor(11, 5.05f/2.0),
        motor_driver(FOC_IN1_PWM, FOC_IN2_PWM, FOC_IN3_PWM, FOC_EN)
    {
        this->voltage_req = voltage;
        this->current_req = current;
        this->rpm_req = rpm;
        this->velocity_req = velocity;
        this->position_req = position;
        this->torque_req = torque;
    }

    void init()
    {
        // Enable motor
        digitalWrite(M_NSLEEP, HIGH);
        // Disable motor sleep
        digitalWrite(M_NRST, HIGH);
        SPI.setMOSI(PA7);
        SPI.setMISO(PA6);
        SPI.setSCLK(PA5);
        SPI.setSSEL(PA4);
        SPI.setClockDivider(SPI_CLOCK_DIV32);
        SPI.setDataMode(SPI_MODE0);
        SPI.setBitOrder(MSBFIRST);

        SPI.begin();
        encoder.init(&SPI);
        motor_driver.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
        motor_driver.voltage_limit = MOTOR_OPERATING_VOLTAGE;
        motor_driver.pwm_frequency = FOC_PWM_FREQ;
        motor_driver.init();
        motor.current_limit = MOTOR_CURRENT_LIMIT;
        motor.torque_controller = TorqueControlType::foc_current;
        motor.controller = MotionControlType::angle;
        motor.PID_velocity.P = 0.5;
        motor.PID_velocity.I = 10;
        motor.PID_velocity.D = 0.0;
        // jerk control using voltage voltage ramp
        // default value is 300 volts per sec  ~ 0.3V per millisecond
        motor.PID_velocity.output_ramp = 300;
        // velocity low pass filtering
        // default 5ms - try different values to see what is the best.
        // the lower the less filtered
        motor.LPF_velocity.Tf = 0.001;
        // angle P controller -  default P=20
        motor.P_angle.P = 20;
        motor.linkSensor(&encoder);
        motor.linkDriver(&motor_driver);
        motor.init();
        Serial3.begin(115200);
        Serial3.println("Motor init");
        motor.useMonitoring(Serial3);
        motor.voltage_sensor_align = MOTOR_OPERATING_VOLTAGE;
        motor.initFOC();
        encoder.update();
        zero_angle = encoder.getSensorAngle();
        motor.monitor_variables |= _MON_ANGLE | _MON_TARGET | _MON_CURR_Q | _MON_VOLT_Q;
    }

    uint16_t get_voltage()
    {
        return motor.voltage.q;
    }

    uint16_t get_current()
    {
        return motor.current.q;
    }

    uint16_t get_rpm()
    {
        return (motor.shaft_velocity / TWO_PI) * 60.0f;
    }

    uint16_t get_velocity()
    {
        return motor.shaft_velocity;
    }

    uint16_t get_position()
    {
        return get_angle();
    }

    uint16_t get_angle()
    {
        // Note there is precision loss here, angle is 14 bits -> float RAD -> 16 bit uint
        // Convert RAD to 16 bit uint
        uint16_t angle = (std::fmod(encoder.getAngle(), TWO_PI) / (TWO_PI)) * PRECISION_16;
        return angle;
    }

    uint16_t get_torque()
    {
        return motor.target;
    }

    void request_position(uint16_t position)
    {
        request_angle(position);
    }

    void request_velocity(uint16_t velocity)
    {
        velocity_req = (float)velocity;
    }

    void request_torque(uint16_t torque)
    {
        torque_req = (float)torque;
    }

    void request_angle(uint16_t angle)
    {
        angle_req = ((float)angle * (TWO_PI)) / PRECISION_16;
    }

    void update()
    {
        motor.monitor();

        motor.loopFOC();
        motor.move(angle_req);
    }
};

#endif // _MOTOR_H_
