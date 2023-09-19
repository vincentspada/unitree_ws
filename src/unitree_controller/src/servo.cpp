/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <string>
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_msgs/msg/motor_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_model/body.hpp"

using namespace std;
using namespace unitree_model;

bool start_up = true;

class MultiThread : public rclcpp::Node
{
public:
    MultiThread(const string &rname) : Node("unitree_gazebo_servo")
    {
        robot_name = rname;
        imu_sub = create_subscription<sensor_msgs::msg::Imu>(
            "/trunk_imu", 1, std::bind(&MultiThread::imuCallback, this, std::placeholders::_1));
        footForce_sub[0] = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FR_foot_contact/the_force", 1, std::bind(&MultiThread::FRfootCallback, this, std::placeholders::_1));
        footForce_sub[1] = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FL_foot_contact/the_force", 1, std::bind(&MultiThread::FLfootCallback, this, std::placeholders::_1));
        footForce_sub[2] = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RR_foot_contact/the_force", 1, std::bind(&MultiThread::RRfootCallback, this, std::placeholders::_1));
        footForce_sub[3] = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RL_foot_contact/the_force", 1, std::bind(&MultiThread::RLfootCallback, this, std::placeholders::_1));
        servo_sub[0] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_hip_controller/state", 1, std::bind(&MultiThread::FRhipCallback, this, std::placeholders::_1));
        servo_sub[1] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, std::bind(&MultiThread::FRthighCallback, this, std::placeholders::_1));
        servo_sub[2] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_calf_controller/state", 1, std::bind(&MultiThread::FRcalfCallback, this, std::placeholders::_1));
        servo_sub[3] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_hip_controller/state", 1, std::bind(&MultiThread::FLhipCallback, this, std::placeholders::_1));
        servo_sub[4] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, std::bind(&MultiThread::FLthighCallback, this, std::placeholders::_1));
        servo_sub[5] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_calf_controller/state", 1, std::bind(&MultiThread::FLcalfCallback, this, std::placeholders::_1));
        servo_sub[6] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_hip_controller/state", 1, std::bind(&MultiThread::RRhipCallback, this, std::placeholders::_1));
        servo_sub[7] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, std::bind(&MultiThread::RRthighCallback, this, std::placeholders::_1));
        servo_sub[8] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_calf_controller/state", 1, std::bind(&MultiThread::RRcalfCallback, this, std::placeholders::_1));
        servo_sub[9] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_hip_controller/state", 1, std::bind(&MultiThread::RLhipCallback, this, std::placeholders::_1));
        servo_sub[10] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, std::bind(&MultiThread::RLthighCallback, this, std::placeholders::_1));
        servo_sub[11] = create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_calf_controller/state", 1, std::bind(&MultiThread::RLcalfCallback, this, std::placeholders::_1));
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        lowState.imu.quaternion[0] = msg->orientation.w;
        lowState.imu.quaternion[1] = msg->orientation.x;
        lowState.imu.quaternion[2] = msg->orientation.y;
        lowState.imu.quaternion[3] = msg->orientation.z;

        lowState.imu.gyroscope[0] = msg->angular_velocity.x;
        lowState.imu.gyroscope[1] = msg->angular_velocity.y;
        lowState.imu.gyroscope[2] = msg->angular_velocity.z;

        lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg->linear_acceleration.z;
    }

    void FRhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[0].mode = msg->mode;
        lowState.motor_state[0].q = msg->q;
        lowState.motor_state[0].dq = msg->dq;
        lowState.motor_state[0].tauEst = msg->tauEst;
    }

    void FRthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[1].mode = msg->mode;
        lowState.motor_state[1].q = msg->q;
        lowState.motor_state[1].dq = msg->dq;
        lowState.motor_state[1].tauEst = msg->tauEst;
    }

    void FRcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[2].mode = msg->mode;
        lowState.motor_state[2].q = msg->q;
        lowState.motor_state[2].dq = msg->dq;
        lowState.motor_state[2].tauEst = msg->tauEst;
    }

    void FLhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[3].mode = msg->mode;
        lowState.motor_state[3].q = msg->q;
        lowState.motor_state[3].dq = msg->dq;
        lowState.motor_state[3].tauEst = msg->tauEst;
    }

    void FLthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[4].mode = msg->mode;
        lowState.motor_state[4].q = msg->q;
        lowState.motor_state[4].dq = msg->dq;
        lowState.motor_state[4].tauEst = msg->tauEst;
    }

    void FLcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[5].mode = msg->mode;
        lowState.motor_state[5].q = msg->q;
        lowState.motor_state[5].dq = msg->dq;
        lowState.motor_state[5].tauEst = msg->tauEst;
    }

    void RRhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[6].mode = msg->mode;
        lowState.motor_state[6].q = msg->q;
        lowState.motor_state[6].dq = msg->dq;
        lowState.motor_state[6].tauEst = msg->tauEst;
    }

    void RRthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[7].mode = msg->mode;
        lowState.motor_state[7].q = msg->q;
        lowState.motor_state[7].dq = msg->dq;
        lowState.motor_state[7].tauEst = msg->tauEst;
    }

    void RRcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[8].mode = msg->mode;
        lowState.motor_state[8].q = msg->q;
        lowState.motor_state[8].dq = msg->dq;
        lowState.motor_state[8].tauEst = msg->tauEst;
    }

    void RLhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[9].mode = msg->mode;
        lowState.motor_state[9].q = msg->q;
        lowState.motor_state[9].dq = msg->dq;
        lowState.motor_state[9].tauEst = msg->tauEst;
    }

    void RLthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[10].mode = msg->mode;
        lowState.motor_state[10].q = msg->q;
        lowState.motor_state[10].dq = msg->dq;
        lowState.motor_state[10].tauEst = msg->tauEst;
    }

    void RLcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[11].mode = msg->mode;
        lowState.motor_state[11].q = msg->q;
        lowState.motor_state[11].dq = msg->dq;
        lowState.motor_state[11].tauEst = msg->tauEst;
    }

    void FRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.eeForce[0].x = msg->wrench.force.x;
        lowState.eeForce[0].y = msg->wrench.force.y;
        lowState.eeForce[0].z = msg->wrench.force.z;
        lowState.footForce[0] = msg->wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.eeForce[1].x = msg->wrench.force.x;
        lowState.eeForce[1].y = msg->wrench.force.y;
        lowState.eeForce[1].z = msg->wrench.force.z;
        lowState.footForce[1] = msg->wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.eeForce[2].x = msg->wrench.force.x;
        lowState.eeForce[2].y = msg->wrench.force.y;
        lowState.eeForce[2].z = msg->wrench.force.z;
        lowState.footForce[2] = msg->wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.eeForce[3].x = msg->wrench.force.x;
        lowState.eeForce[3].y = msg->wrench.force.y;
        lowState.eeForce[3].z = msg->wrench.force.z;
        lowState.footForce[3] = msg->wrench.force.z;
    }

private:
    string robot_name;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr footForce_sub[4];
    rclcpp::Subscription<unitree_legged_msgs::msg::MotorState>::SharedPtr servo_sub[12];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    string robot_name;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("unitree_gazebo_servo");
    node->get_parameter_or<string>("/robot_name", robot_name, "");
    cout << "robot_name: " << robot_name << endl;

    MultiThread listen_publish_obj(robot_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
