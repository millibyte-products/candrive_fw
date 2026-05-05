#include "motor.h"
#include "device.h"

#include <stdint.h>
#include <drivers/BLDCDriver3PWM.h>
#include <drivers/hardware_specific/stm32/stm32_mcu.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <BLDCMotor.h>
#include <Arduino.h>

HardwareSerial Serial3(PB11, PB10); 

#define FOC_PWM_FREQ (50000)
#define FTOUINT(x,limit) ((x / limit) * PRECISION_16)
static const float PRECISION_16 = 65536.0f;

static MagneticSensorMT6701SSI encoder(ENCODER_CSN);
static BLDCMotor motor(12.0f, 5.65f);
static BLDCDriver3PWM motor_driver(FOC_IN1_PWM, FOC_IN2_PWM, FOC_IN3_PWM, FOC_EN);
static float angle_req = 0.0f;
static float zero_angle = 0.0f;
static float torque_req = 0.0f;
static float velocity_req = 0.0f;

#define MOTOR_MAX_TORQUE (2000.0f)
// In radians per second
#define MOTOR_MAX_VELOCITY (600.0f * TWO_PI)
#define MOTOR_MAX_RPM (600.0f)

void motor_init()
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
    motor.initFOC();
    encoder.update();
    zero_angle = encoder.getSensorAngle();
    motor.monitor_variables |= _MON_ANGLE | _MON_TARGET | _MON_VOLT_Q;
}

void motor_update()
{
    motor.monitor();
    motor.loopFOC();
    motor.move(angle_req);
}

uint16_t motor_get_voltage()
{
    return FTOUINT(motor.voltage.q, MOTOR_SUPPLY_VOLTAGE);
}

uint16_t motor_get_current()
{
    return FTOUINT(motor.current.q, MOTOR_CURRENT_LIMIT);
}

uint16_t motor_get_rpm()
{
    return FTOUINT((motor.shaft_velocity / TWO_PI) * 60.0f, MOTOR_MAX_RPM);
}

uint16_t motor_get_velocity()
{
    return FTOUINT(motor.shaft_velocity, MOTOR_MAX_VELOCITY);
}

uint16_t motor_get_position()
{
    return motor_get_angle();
}

uint16_t motor_get_angle()
{
    // Note there is precision loss here, angle is 14 bits -> float RAD -> 16 bit uint
    // Convert RAD to 16 bit uint
    uint16_t angle = FTOUINT(encoder.getAngle(), TWO_PI);
    return angle;
}

void motor_request_angle(uint16_t angle)
{
    angle_req = ((float)angle * (TWO_PI)) / PRECISION_16;
}

uint16_t motor_get_torque()
{
    return FTOUINT(motor.target, MOTOR_MAX_TORQUE);
}

void motor_request_position(uint16_t position)
{
    motor_request_angle(position);
}

void motor_request_velocity(uint16_t velocity)
{
    velocity_req = (float)velocity;
}

void motor_request_torque(uint16_t torque)
{
    torque_req = (float)torque;
}

