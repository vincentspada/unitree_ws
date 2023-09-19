/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef LAIKAGO_CONTROL_TOOL_HPP_
#define LAIKAGO_CONTROL_TOOL_HPP_

#include <cstdint>
#include <algorithm>
#include <cmath>

#define posStopF (2.146E+9)  // stop position control mode
#define velStopF (16000.0)   // stop velocity control mode

struct ServoCmd
{
    uint8_t mode;
    double pos;
    double posStiffness;
    double vel;
    double velStiffness;
    double torque;
};

double clamp(double&, double, double);  // e.g., clamp(1.5, -1, 1) = 1
double computeVel(double current_position, double last_position, double last_velocity, double duration);  // get current velocity
double computeTorque(double current_position, double current_velocity, ServoCmd&);  // get torque

#endif
