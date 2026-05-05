#ifndef _DEVICE_H_
#define _DEVICE_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <stdint.h>

#include "protocol.h"

// Physical parameters
#define MOTOR_SUPPLY_VOLTAGE (12.0f)
#define MOTOR_CURRENT_LIMIT (1.0f)
#define MOTOR_OPERATING_VOLTAGE (12.0f)
#define HOMING_TIMEOUT_MS (5000)

#define LED_FREQ (100000) // 100khz
#define SERVO_FREQ (50) // 50hz

// PINMAP
#define ENDSTOP0 (PC15)
#define ENDSTOP1 (PD1)

#define A0 (PA0)
#define A1 (PA1)

#define FOC_IN2_PWM (PA2) // TIM2 CH3
#define FOC_IN3_PWM (PB3) // TIM2 CH2
#define FOC_IN1_PWM (PA15) // TIM2 CH1
#define FOC_EN (PA3)

// TIM
#define LED_SYS_PWM (PB4) // TIM3 CH1
#define LED_STAT_PWM (PB5) // TIM3 CH2

#define SRV0_PWM (PB6) // TIM4 CH1
#define SRV1_PWM (PB7) // TIM4 CH2

#define M_NFAULT (PA10)
#define M_NSLEEP (PA9)
#define M_NRST (PA8)

#define ENCODER_CSN (PA4)

#define MISC (PB0)

#define PWMCHANNEL(x) STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(x)))
#define INVALID_DEVICE (-1)

#define LED_DUTY_DEFAULT (25)
#define FOC_PWM_FREQ (50000)

#define DISCOVERY_INTERVAL_MS (15000)

typedef enum
{
    RESET_REASON_UNKNOWN = 0,
    RESET_REASON_LOW_POWER_RESET,
    RESET_REASON_WINDOW_WATCHDOG_RESET,
    RESET_REASON_INDEPENDENT_WATCHDOG_RESET,
    RESET_REASON_SOFTWARE_RESET,
    RESET_REASON_POWER_ON_POWER_DOWN_RESET,
    RESET_REASON_EXTERNAL_RESET_PIN_RESET,
    RESET_REASON_BROWNOUT_RESET,
} reset_reason_t;

typedef enum
{
    OPERATING_MODE_DISCOVERY = 0,
    OPERATING_MODE_CONTROL,
} operating_mode_t;

void device_init();
void device_update();
void reset_interfaces();

int16_t device_id_get();
int16_t cached_id_get();
void device_id_set(int16_t value);
void cached_id_set(int16_t value);
uint32_t serial_no_get();
uint8_t fw_major_get();
uint8_t fw_minor_get();
uint8_t fw_patch_get();

// Build info
uint64_t build_ts_get();
const uint8_t *build_commit_get(); // 20 bytes for git sha1
uint8_t protocol_ver_get();
uint8_t hw_ver_get();

reset_reason_t reset_reason_get(void);

// Device control
void operating_mode_set(operating_mode_t mode);
operating_mode_t operating_mode_get();

void servo_set(servo_address_t s, uint16_t duty);
uint16_t servo_get(servo_address_t s);

void led_set(led_address_t l, uint8_t duty);
uint8_t led_get(led_address_t l);

#endif // _DEVICE_H_
