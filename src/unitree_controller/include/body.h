/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef UNITREE_MODEL_BODY_H_
#define UNITREE_MODEL_BODY_H_

#include "rclcpp/rclcpp.hpp"
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/high_state.hpp"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern std::vector<rclcpp::Publisher<unitree_legged_msgs::msg::LowCmd>::SharedPtr> servo_pub;
extern rclcpp::Publisher<unitree_legged_msgs::msg::HighState>::SharedPtr highState_pub;
extern unitree_legged_msgs::msg::LowCmd lowCmd;
extern unitree_legged_msgs::msg::LowState lowState;

void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(std::vector<double>& jointPositions, double duration);
}

#endif