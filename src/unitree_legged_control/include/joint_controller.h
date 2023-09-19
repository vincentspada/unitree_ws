/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef UNITREE_ROS_JOINT_CONTROLLER_HPP_
#define UNITREE_ROS_JOINT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <unitree_legged_msgs/msg/motor_cmd.hpp>
#include <unitree_legged_msgs/msg/motor_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <unitree_joint_control_tool.hpp>

namespace unitree_legged_control
{
    class UnitreeJointController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    private:
        hardware_interface::JointHandle joint;
        rclcpp::Subscription<unitree_legged_msgs::msg::MotorCmd>::SharedPtr sub_cmd;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft;
        // rclcpp::Publisher<unitree_legged_msgs::msg::MotorState>::SharedPtr pub_state;
        control_toolbox::Pid pid_controller_;
        std::shared_ptr<realtime_tools::RealtimePublisher<unitree_legged_msgs::msg::MotorState>> controller_state_publisher_;

    public:
        // bool start_up;
        std::string name_space;
        std::string joint_name;
        float sensor_torque;
        bool isHip, isThigh, isCalf, rqtTune;
        urdf::JointConstSharedPtr joint_urdf;
        realtime_tools::RealtimeBuffer<unitree_legged_msgs::msg::MotorCmd> command;
        unitree_legged_msgs::msg::MotorCmd lastCmd;
        unitree_legged_msgs::msg::MotorState lastState;
        ServoCmd servoCmd;

        UnitreeJointController();
        ~UnitreeJointController();
        controller_interface::return_type init(hardware_interface::EffortJointInterface *robot, const std::string &controller_name);
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period);
        void starting(const rclcpp::Time &time);
        void stopping();
        void setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void setCommandCB(const unitree_legged_msgs::msg::MotorCmd::SharedPtr msg);
        void positionLimits(double &position);
        void velocityLimits(double &velocity);
        void effortLimits(double &effort);

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    };
}

#endif
