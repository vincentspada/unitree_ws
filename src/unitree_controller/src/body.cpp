/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_model/body.hpp"

namespace unitree_model {

std::vector<rclcpp::Publisher<unitree_legged_msgs::msg::LowCmd>::SharedPtr> servo_pub(12);
unitree_legged_msgs::msg::LowCmd lowCmd;
unitree_legged_msgs::msg::LowState lowState;

// These parameters are only for reference.
// Actual parameters need to be debugged if you want to run on a real robot.
void paramInit()
{
    for (int i = 0; i < 4; i++) {
        lowCmd.motor_cmd[i * 3 + 0].mode = 0x0A;
        lowCmd.motor_cmd[i * 3 + 0].kp = 70;
        lowCmd.motor_cmd[i * 3 + 0].dq = 0;
        lowCmd.motor_cmd[i * 3 + 0].kd = 3;
        lowCmd.motor_cmd[i * 3 + 0].tau = 0;
        lowCmd.motor_cmd[i * 3 + 1].mode = 0x0A;
        lowCmd.motor_cmd[i * 3 + 1].kp = 180;
        lowCmd.motor_cmd[i * 3 + 1].dq = 0;
        lowCmd.motor_cmd[i * 3 + 1].kd = 8;
        lowCmd.motor_cmd[i * 3 + 1].tau = 0;
        lowCmd.motor_cmd[i * 3 + 2].mode = 0x0A;
        lowCmd.motor_cmd[i * 3 + 2].kp = 300;
        lowCmd.motor_cmd[i * 3 + 2].dq = 0;
        lowCmd.motor_cmd[i * 3 + 2].kd = 15;
        lowCmd.motor_cmd[i * 3 + 2].tau = 0;
    }
    for (int i = 0; i < 12; i++) {
        lowCmd.motor_cmd[i].q = lowState.motor_state[i].q;
    }
}

void stand()
{
    std::vector<double> pos = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                               0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2 * 1000);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    for (int m = 0; m < 12; m++) {
        servo_pub[m]->publish(lowCmd.motor_cmd[m]);
    }
    rclcpp::spin_some(rclcpp::contexts::default_context::get_global_default_context());
    usleep(1000);
}

void moveAllPosition(std::vector<double>& targetPos, double duration)
{
    std::vector<double> pos(12), lastPos(12);
    double percent;
    for (int j = 0; j < 12; j++) lastPos[j] = lowState.motor_state[j].q;
    for (int i = 1; i <= duration; i++) {
        if (!rclcpp::ok()) break;
        percent = (double)i / duration;
        for (int j = 0; j < 12; j++) {
            lowCmd.motor_cmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
        }
        sendServoCmd();
    }
}

} // namespace unitree_model
