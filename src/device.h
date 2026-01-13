#ifndef _DEVICE_H_
#define _DEVICE_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <Arduino.h>
#include <stdint.h>

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
#define INVALID_DEVICE (0)

uint8_t get_device_id();
void set_device_id(uint8_t value);
uint32_t get_serial_no();
uint32_t get_fw_ver();
// Seconds since epoch
uint64_t get_build_ts();
// 20 bytes for git sha1
uint8_t* get_build_commit();
uint8_t get_protocol_ver();
uint32_t get_hw_ver();

#endif // _DEVICE_H_
