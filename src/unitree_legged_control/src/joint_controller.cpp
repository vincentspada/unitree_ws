/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_control/joint_controller.hpp"
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/joint_state_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#define PMSM      (0x0A)
#define BRAKE     (0x00)
#define PosStopF  (2.146E+9)
#define VelStopF  (16000.0)

namespace unitree_legged_control 
{

    UnitreeJointController::UnitreeJointController()
    {
        memset(&lastCmd, 0, sizeof(unitree_legged_msgs::msg::MotorCmd));
        memset(&lastState, 0, sizeof(unitree_legged_msgs::msg::MotorState));
        memset(&servoCmd, 0, sizeof(ServoCmd));
    }

    UnitreeJointController::~UnitreeJointController()
    {
        sub_ft.reset();
        sub_cmd.reset();
    }

    void UnitreeJointController::setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        if(isHip) sensor_torque = msg->wrench.torque.x;
        else sensor_torque = msg->wrench.torque.y;
    }

    void UnitreeJointController::setCommandCB(const unitree_legged_msgs::msg::MotorCmd::SharedPtr msg)
    {
        lastCmd.mode = msg->mode;
        lastCmd.q = msg->q;
        lastCmd.Kp = msg->Kp;
        lastCmd.dq = msg->dq;
        lastCmd.Kd = msg->Kd;
        lastCmd.tau = msg->tau;
        command.writeFromNonRT(lastCmd);
    }

    // Controller initialization
    controller_interface::return_type UnitreeJointController::init(const std::string& controller_name)
    {
        isHip = false;
        isThigh = false;
        isCalf = false;
        sensor_torque = 0;

        if (!get_node()->get_parameter("joint", joint_name))
        {
            RCLCPP_ERROR(get_logger(), "No joint given in parameter 'joint'");
            return controller_interface::return_type::ERROR;
        }

        if (auto urdf = get_node()->get_parameter("robot_description"))
        {
            urdf::Model model;
            model.initString(urdf.as_string());
            if (!model.getJoint(joint_name, joint_urdf))
            {
                RCLCPP_ERROR(get_logger(), "Could not find joint '%s' in URDF", joint_name.c_str());
                return controller_interface::return_type::ERROR;
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to get parameter 'robot_description'");
            return controller_interface::return_type::ERROR;
        }

        if(joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" || joint_name == "RR_hip_joint" || joint_name == "RL_hip_joint"){
            isHip = true;
        }
        if(joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" || joint_name == "RR_calf_joint" || joint_name == "RL_calf_joint"){
            isCalf = true;
        }

        try
        {
            joint = hardware_interface::JointHandle(joint_name, "position");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get joint handle: %s", e.what());
            return controller_interface::return_type::ERROR;
        }

        sub_ft = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "joint_wrench", 1, std::bind(&UnitreeJointController::setTorqueCB, this, std::placeholders::_1));

        sub_cmd = create_subscription<unitree_legged_msgs::msg::MotorCmd>(
            "command", 20, std::bind(&UnitreeJointController::setCommandCB, this, std::placeholders::_1));

        controller_state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<unitree_legged_msgs::msg::MotorState>>(
            get_node(), name_space + "/state", 1);

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type UnitreeJointController::update()
    {
        double currentPos, currentVel, calcTorque;
        lastCmd = *(command.readFromRT());

        // set command data
        if(lastCmd.mode == PMSM)
        {
            servoCmd.pos = lastCmd.q;
            positionLimits(servoCmd.pos);
            servoCmd.posStiffness = lastCmd.Kp;
            if(std::fabs(lastCmd.q - PosStopF) < 0.00001){
                servoCmd.posStiffness = 0;
            }
            servoCmd.vel = lastCmd.dq;
            velocityLimits(servoCmd.vel);
            servoCmd.velStiffness = lastCmd.Kd;
            if(std::fabs(lastCmd.dq - VelStopF) < 0.00001){
                servoCmd.velStiffness = 0;
            }
            servoCmd.torque = lastCmd.tau;
            effortLimits(servoCmd.torque);
        }
        if(lastCmd.mode == BRAKE)
        {
            servoCmd.posStiffness = 0;
            servoCmd.vel = 0;
            servoCmd.velStiffness = 20;
            servoCmd.torque = 0;
            effortLimits(servoCmd.torque);
        }

        currentPos = joint.get_position();
        currentVel = computeVel(currentPos, static_cast<double>(lastState.q), static_cast<double>(lastState.dq), get_period().toSec());
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);      
        effortLimits(calcTorque);

        joint.set_position(currentPos, calcTorque);

        lastState.q = currentPos;
        lastState.dq = currentVel;
        lastState.tauEst = joint.get_effort();

        if (controller_state_publisher_ && controller_state_publisher_->trylock())
        {
            controller_state_publisher_->msg_.q = lastState.q;
            controller_state_publisher_->msg_.dq = lastState.dq;
            controller_state_publisher_->msg_.tauEst = lastState.tauEst;
            controller_state_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type UnitreeJointController::starting(const rclcpp::Time& time)
    {
        double init_pos = joint.get_position();
        lastCmd.q = init_pos;
        lastState.q = init_pos;
        lastCmd.dq = 0;
        lastState.dq = 0;
        lastCmd.tau = 0;
        lastState.tauEst = 0;
        command.initRT(lastCmd);

        pid_controller_.reset();

        return controller_interface::return_type::OK;
    }

    void UnitreeJointController::stopping(const rclcpp::Time& time)
    {
        // Nothing to do here since this is a position controller.
    }

    void UnitreeJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void UnitreeJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void UnitreeJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerBase);
