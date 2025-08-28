#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

/* Copyright (C) 2025 Austen Danger Bartels - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the CC BY-NC-SA license.
 *
 * You should have received a copy of the CC BY-NC-SA license with
 * this file. If not, please visit : github.com/millibyte/candrive-fw
 */

#include <stdint.h>

class KinematicsModel
{
private:
    float zero_angle;
    float angle_ratio;
    float low_pass;
    float pid_p;
    float position_req;
    float current_voltage_req;
public:
    KinematicsModel(float rail = 125.0f, float stroke = 120.0f, float pulley_circumference = 60.0f, float p = 0.002f, float lp_term = 0.8f)
    {
        angle_ratio = (pulley_circumference) / (TWO_PI * stroke);
        zero_angle = 0;
        position_req = 0;
    }
    
    float set_zero(float angle)
    {
        zero_angle = angle;
    }

    float set_position_req(float p)
    {
        position_req = p;
    }

    float calculate_voltage(float angle)
    {
        // TODO: correct for rotations/slipping
        float position = (angle - zero_angle) * angle_ratio;
        float voltage = pid_p * (position_req - position);
        current_voltage_req = (low_pass * current_voltage_req) + ((1.0f - low_pass) * voltage);
        return current_voltage_req;
    }
};

#endif // _KINEMATICS_H_
