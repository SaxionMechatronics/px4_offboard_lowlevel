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

#ifndef CONTROLLER_CONTROLLER_H
#define CONTROLLER_CONTROLLER_H

#include <eigen3/Eigen/Eigen>

class controller {
public:
    controller();
    void calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion);

    // Setters
    void setOdometry(const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_B_W, 
        const Eigen::Vector3d &velocity_B, const Eigen::Vector3d &angular_velocity_B){
        R_B_W_ = orientation_B_W.toRotationMatrix();
        position_W_ = position_W;
        velocity_W_ = R_B_W_ * velocity_B;
        angular_velocity_B_ = angular_velocity_B;
    }

    void setTrajectoryPoint(const Eigen::Vector3d &position_W, const Eigen::Vector3d &velocity_W, const Eigen::Vector3d &acceleration_W
                    , const Eigen::Quaterniond &orientation_W, const Eigen::Vector3d &angular_velocity_B){
        r_position_W_ = position_W;
        r_velocity_W_ = velocity_W;
        r_acceleration_W_ = acceleration_W;
        r_R_B_W_ = orientation_W.toRotationMatrix();
        r_yaw = r_R_B_W_.eulerAngles(0, 1, 2)(2);
        r_yaw_rate = angular_velocity_B(2);
    }

    void setTrajectoryPoint(const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_W){
        r_position_W_ = position_W;
        r_velocity_W_.setZero();
        r_acceleration_W_.setZero();
        r_R_B_W_ = orientation_W.toRotationMatrix();
        r_yaw = r_R_B_W_.eulerAngles(0, 1, 2)(2);
        r_yaw_rate = 0.0;
    }

    void setKPositionGain(const Eigen::Vector3d &PositionGain){
        position_gain_ = PositionGain;
    }

    void setKVelocityGain(const Eigen::Vector3d &VelocityGain){
        velocity_gain_ = VelocityGain;
    }

    void setKAttitudeGain(const Eigen::Vector3d &AttitudeGain){
        attitude_gain_ = AttitudeGain;
    }

    void setKAngularRateGain(const Eigen::Vector3d AngularRateGain){
        angular_rate_gain_ = AngularRateGain;
    }

    void setUavMass(double uavMass) {
        _uav_mass = uavMass;
    }

    void setInertiaMatrix(const Eigen::Vector3d &inertiaMatrix) {
        _inertia_matrix = inertiaMatrix;
    }

    void setGravity(double gravity) {
        _gravity = gravity;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // UAV Parameter
    double _uav_mass;
    Eigen::Vector3d _inertia_matrix;
    double _gravity;
    
    // Lee Controller Gains
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d angular_rate_gain_;
    
    // Current states
    Eigen::Vector3d position_W_;
    Eigen::Vector3d velocity_W_;
    Eigen::Matrix3d R_B_W_;
    Eigen::Vector3d angular_velocity_B_;
    // References
    Eigen::Vector3d r_position_W_;
    Eigen::Vector3d r_velocity_W_;
    Eigen::Vector3d r_acceleration_W_;
    Eigen::Matrix3d r_R_B_W_;
    double r_yaw;
    double r_yaw_rate;
};

#endif //CONTROLLER_CONTROLLER_H
