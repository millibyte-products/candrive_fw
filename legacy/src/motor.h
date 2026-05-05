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

void motor_init();
void motor_update();

uint16_t motor_get_voltage();
uint16_t motor_get_current();
uint16_t motor_get_rpm();
uint16_t motor_get_velocity();
uint16_t motor_get_position();
uint16_t motor_get_angle();
uint16_t motor_get_torque();

void motor_request_position(uint16_t position);
void motor_request_velocity(uint16_t velocity);
void motor_request_torque(uint16_t torque);
void motor_request_angle(uint16_t angle);

#endif // _MOTOR_H_
