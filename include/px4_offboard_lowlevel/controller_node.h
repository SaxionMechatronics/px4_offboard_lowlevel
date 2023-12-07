/****************************************************************************
 *
 *   Copyright (c) 2023, SMART Research Group, Saxion University of 
 *   Applied Sciences.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef CONTROLLER_CONTROLLER_NODE_H
#define CONTROLLER_CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <string>

#include "px4_offboard_lowlevel/controller.h"

#include <chrono>
using namespace std::chrono_literals;

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();    
    //virtual ~controller_node();
    void updateControllerOutput();

private:

    controller controller_;

    // Timers
    rclcpp::TimerBase::SharedPtr controllerTimer;
    rclcpp::TimerBase::SharedPtr offboardTimer;

    // subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_sub_;
    
    // Publishers
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Services
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    
    // Messages
    px4_msgs::msg::VehicleAttitudeSetpoint attitude_setpoint_msg;
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;

    // Topics namescontroller_
    std::string command_pose_topic_;
    std::string command_traj_topic_;
    std::string odometry_topic_;
    std::string status_topic_;
    std::string battery_status_topic_;
    std::string actuator_status_topic;
    std::string offboard_control_topic_;
    std::string vehicle_command_topic_;
    std::string attitude_setpoint_topic_;
    std::string thrust_setpoint_topic_;
    std::string torque_setpoint_topic_;
    std::string actuator_control_topic_;

    // UAV Parameters
    double _arm_length;
    int _num_of_arms;
    double _moment_constant;
    double _thrust_constant;
    double _max_rotor_speed;
    Eigen::Vector3d _omega_to_pwm_coefficients;
    int _PWM_MIN;
    int _PWM_MAX;
    int _input_scaling;
    int _zero_position_armed;
    Eigen::MatrixXd torques_and_thrust_to_rotor_velocities_;
    Eigen::MatrixXd throttles_to_normalized_torques_and_thrust_;

    // Controller gains
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d ang_vel_gain_;

    // Logic switches
    int control_mode_;
    bool in_sitl_mode_;
    
    px4_msgs::msg::VehicleStatus current_status_;
    bool connected_ = false;

    void loadParams();
    void secureConnection();
    void arm();
    void disarm();

    // CallBacks
    void commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void commandTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr& traj_msg);
    void vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg);

    void publishActuatorMotorsMsg(const Eigen::VectorXd& throttles);
    void publishThrustTorqueMsg(const Eigen::Vector4d& controller_output);
    void publishAttitudeSetpointMsg(const Eigen::Vector4d& controller_output, const Eigen::Quaterniond& desired_quaternion);
    void publishOffboardControlModeMsg();
    void publish_vehicle_command(uint16_t command, float param1 =0.0, float param2 = 0.0);   

    void compute_ControlAllocation_and_ActuatorEffect_matrices();
    void px4Inverse (Eigen::Vector4d *normalized_torque_and_thrust, Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench);
    void px4InverseSITL (Eigen::Vector4d *normalized_torque_and_thrust, Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench);

    inline Eigen::Vector3d rotateVectorFromToENU_NED(const Eigen::Vector3d& vec_in) {
        // NED (X North, Y East, Z Down) & ENU (X East, Y North, Z Up)
        Eigen::Vector3d vec_out;
        vec_out << vec_in[1], vec_in[0], -vec_in[2];
        return vec_out;
    }

    inline Eigen::Vector3d rotateVectorFromToFRD_FLU(const Eigen::Vector3d& vec_in) {
        // FRD (X Forward, Y Right, Z Down) & FLU (X Forward, Y Left, Z Up)
        Eigen::Vector3d vec_out;
        vec_out << vec_in[0], -vec_in[1], -vec_in[2];
        return vec_out;
    }

    inline Eigen::Quaterniond rotateQuaternionFromToENU_NED(const Eigen::Quaterniond& quat_in) {
        // Transform from orientation represented in ROS format to PX4 format and back
        //  * Two steps conversion:
        //  * 1. aircraft to NED is converted to aircraft to ENU (NED_to_ENU conversion)
        //  * 2. aircraft to ENU is converted to baselink to ENU (baselink_to_aircraft conversion)
        // OR 
        //  * 1. baselink to ENU is converted to baselink to NED (ENU_to_NED conversion)
        //  * 2. baselink to NED is converted to aircraft to NED (aircraft_to_baselink conversion
        // NED_ENU_Q Static quaternion needed for rotating between ENU and NED frames
        Eigen::Vector3d euler_1(M_PI, 0.0, M_PI_2);
        Eigen::Quaterniond NED_ENU_Q(Eigen::AngleAxisd(euler_1.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler_1.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler_1.x(), Eigen::Vector3d::UnitX()));
        
        // AIRCRAFT_BASELINK_Q Static quaternion needed for rotating between aircraft and base_link frames
        Eigen::Vector3d euler_2(M_PI, 0.0, 0.0);
        Eigen::Quaterniond AIRCRAFT_BASELINK_Q(Eigen::AngleAxisd(euler_2.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler_2.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler_2.x(), Eigen::Vector3d::UnitX()));
        
        return (NED_ENU_Q*quat_in)*AIRCRAFT_BASELINK_Q;
    }

    inline void eigenOdometryFromPX4Msg(const px4_msgs::msg::VehicleOdometry::SharedPtr msg,
                                Eigen::Vector3d& position_W, Eigen::Quaterniond& orientation_B_W,
                                Eigen::Vector3d& velocity_B, Eigen::Vector3d& angular_velocity_B) {

        position_W = rotateVectorFromToENU_NED(Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]));

        Eigen::Quaterniond quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        orientation_B_W = rotateQuaternionFromToENU_NED(quaternion);

        velocity_B = rotateVectorFromToENU_NED(Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]));

        angular_velocity_B = rotateVectorFromToFRD_FLU
                                (Eigen::Vector3d(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));
    }

    inline void eigenTrajectoryPointFromMsg(
        const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr& msg,
        Eigen::Vector3d& position_W, Eigen::Quaterniond& orientation_W_B,
        Eigen::Vector3d& velocity_W, Eigen::Vector3d& angular_velocity_W,
        Eigen::Vector3d& acceleration_W) {
        
        if (msg->transforms.empty()) {
            return;
        }

        position_W << msg->transforms[0].translation.x,
            msg->transforms[0].translation.y,
            msg->transforms[0].translation.z;
        Eigen::Quaterniond quaternion(msg->transforms[0].rotation.w,
                                    msg->transforms[0].rotation.x,
                                    msg->transforms[0].rotation.y,
                                    msg->transforms[0].rotation.z);
        orientation_W_B = quaternion;
        if (msg->velocities.size() > 0) {
            velocity_W << msg->velocities[0].linear.x,
                msg->velocities[0].linear.y,
                msg->velocities[0].linear.z;
            angular_velocity_W << msg->velocities[0].angular.x,
                msg->velocities[0].angular.y,
                msg->velocities[0].angular.z;
        } else {
            velocity_W.setZero();
            angular_velocity_W.setZero();
        }
        if (msg->accelerations.size() > 0) {
            acceleration_W << msg->accelerations[0].linear.x,
                msg->accelerations[0].linear.y,
                msg->accelerations[0].linear.z;
        } else {
            acceleration_W.setZero();
        }
    }

    inline void eigenTrajectoryPointFromPoseMsg(
        const geometry_msgs::msg::PoseStamped::SharedPtr& msg, Eigen::Vector3d& position_W, Eigen::Quaterniond& orientation_W_B) {

        position_W << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond quaternion(msg->pose.orientation.w,
                                    msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z);
        orientation_W_B = quaternion;
    }
};


#endif //px4_offboard_lowlevel_CONTROLLER_NODE_H
