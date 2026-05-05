#include "device.h"

// Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
// You may use, distribute and modify this code under the
// terms of the CC BY-NC-SA license.
//
// You should have received a copy of the CC BY-NC-SA license with
// this file. If not, please visit : github.com/millibyte/candrive-fw

// EXTERNAL MACROS (set by build system)
// #define BUILD_COMMIT "unknown"
// #define BUILD_TS 1665072400
// #define VERSION_MAJOR 0
// #define VERSION_MINOR 9
// #define VERSION_PATCH 0

#include "build_info.h"
#include "protocol.h"
#include "motor.h"
#include "errors.h"

#include <Arduino.h>

// Maps to upper 8 bits of CAN ID
static int16_t device_id = INVALID_DEVICE;
static int16_t cached_id = 0;
static reset_reason_t last_reason = RESET_REASON_UNKNOWN;
static uint16_t ser0_duty;
static uint16_t ser1_duty;
static uint8_t sys_led_duty;
static uint8_t stat_led_duty;
static HardwareTimer led_timer(TIM3);
static HardwareTimer servo_timer(TIM4);
static operating_mode_t operating_mode = OPERATING_MODE_DISCOVERY;

reset_reason_t read_reset_reason(void)
{
    reset_reason_t reset_reason = RESET_REASON_UNKNOWN;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_reason = RESET_REASON_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_reason = RESET_REASON_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_reason = RESET_REASON_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        // NVIC_SystemReset()
        reset_reason = RESET_REASON_SOFTWARE_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_reason = RESET_REASON_POWER_ON_POWER_DOWN_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_reason = RESET_REASON_EXTERNAL_RESET_PIN_RESET;
    }

    // Clear all the reset flags or else they will remain set during future
    // resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_reason;
}

void reset_interfaces()
{
    servo_set(SERVO_0, 0);
    servo_set(SERVO_1, 0);
    led_set(LED_SYS, 0);
    led_set(LED_STAT, 0);
}

void device_init()
{
    device_id = INVALID_DEVICE;
    last_reason = read_reset_reason();
}

int16_t device_id_get()
{
    return device_id;
}

int16_t cached_id_get()
{
    return cached_id;
}

void cached_id_set(int16_t value)
{
    cached_id = value;
}

void device_id_set(int16_t value)
{
    device_id = value;
}

uint32_t serial_no_get()
{
    // TODO read/write to OTP
    return 0;
}
uint8_t fw_major_get()
{
    // Set by build system
    return VERSION_MAJOR;
}

uint8_t fw_minor_get()
{
    // Set by build system
    return VERSION_MINOR;
}

uint8_t fw_patch_get()
{
    // Set by build system
    return VERSION_PATCH;
}

uint64_t build_ts_get()
{
    // Set by build system
    return BUILD_TIMESTAMP;
}

const uint8_t *build_commit_get()
{
    // Set by build system
    return GIT_SHA;
}

uint8_t protocol_ver_get()
{
    // Read from fw
    return 1;
}

uint8_t hw_ver_get()
{
    // Read from flash metadata
    return 0;
}

reset_reason_t reset_reason_get(void)
{
    return last_reason;
}

void operating_mode_set(operating_mode_t mode)
{
    operating_mode = mode;
    reset_interfaces();
}

operating_mode_t operating_mode_get()
{
    return operating_mode;
}

void set_pwm(HardwareTimer *timer, int pin, uint16_t duty, uint32_t freq)
{
    if (timer)
    {
        uint32_t computed_duty = ((uint32_t)duty * freq) / 65535;
        uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
        timer->setPWM(channel, pin, freq, computed_duty);
    }
}

void led_set(led_address_t l, uint8_t duty)
{
    int pin;
    switch ((uint8_t)l)
    {
    case LED_SYS:
        sys_led_duty = duty;
        pin = LED_SYS_PWM;
        break;
    case LED_STAT:
        stat_led_duty = duty;
        pin = LED_STAT_PWM;
        break;
    default:
        return;
    }
    uint32_t computed_duty = ((uint32_t)duty * LED_FREQ) / 255;
    set_pwm(&led_timer, pin, computed_duty, LED_FREQ);
}

uint8_t led_get(led_address_t l)
{
    uint8_t duty = 0;
    if ((uint8_t)l == LED_SYS)
    {
        duty = sys_led_duty;
    }
    else if (l == LED_STAT)
    {
        duty = stat_led_duty;
    }
    return duty;
}

void servo_set(servo_address_t s, uint16_t duty)
{
    int pin;
    switch (s)
    {
    case SERVO_0:
        ser0_duty = duty;
        pin = SRV0_PWM;
        break;
    case SERVO_1:
        ser1_duty = duty;
        pin = SRV1_PWM;
        break;
    default:
        return;
    }
    uint32_t computed_duty = ((uint32_t)duty * SERVO_FREQ) / 65535;
    set_pwm(&servo_timer, pin, computed_duty, SERVO_FREQ);
}

uint16_t servo_get(servo_address_t s)
{
    uint16_t duty = 0;
    switch (s)
    {
    case SERVO_0:

        duty = ser0_duty;
        break;
    case SERVO_1:
        duty = ser1_duty;
        break;
    default:
        return 0;
    }
    return duty;
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

void device_update()
{
    static uint32_t soft_timer = 0;
    switch (operating_mode_get())
    {
    case OPERATING_MODE_DISCOVERY:
        if ((millis() - soft_timer) > DISCOVERY_INTERVAL_MS)
        {
            send_discovery_query();
            soft_timer = millis();
        }

        if (millis() / 1000 % 2 == 0)
        {
            led_set(LED_SYS, 25);
            led_set(LED_STAT, 25);
        }
        else
        {
            led_set(LED_SYS, 0);
            led_set(LED_STAT, 25);
        }
        break;
    case OPERATING_MODE_CONTROL:
        led_set(LED_STAT, LED_DUTY_DEFAULT);
        led_set(LED_SYS, sys_led_duty);
        servo_set(SERVO_0, ser0_duty);
        servo_set(SERVO_1, ser1_duty);
        break;
    default:
        break;
    }
}
