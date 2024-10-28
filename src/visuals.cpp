#include "../include/px4_offboard_lowlevel/controller.h"
#include <fstream>
#include <iomanip>  // For setting the precision

controller::controller(){
    // Initialize any necessary variables here.
}

void controller::calculateControllerOutput(
        Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) {
    assert(controller_torque_thrust);

    controller_torque_thrust->resize(4);

    // Geometric controller based on:
    // T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),
    // " 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010.

    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p = position_W_ - r_position_W_;
    const Eigen::Vector3d e_v = velocity_W_ - r_velocity_W_;

    const Eigen::Vector3d I_a_d = -position_gain_.cwiseProduct(e_p)
                                -velocity_gain_.cwiseProduct(e_v)
                                +_uav_mass * _gravity * Eigen::Vector3d::UnitZ() + _uav_mass * r_acceleration_W_;
    thrust = I_a_d.dot(R_B_W_.col(2));

    Eigen::Vector3d B_z_d = I_a_d.normalized();

    // Calculate Desired Rotational Matrix
    const Eigen::Vector3d B_x_d(std::cos(r_yaw), std::sin(r_yaw), 0.0);
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d).normalized();
    R_d_w.col(0) = B_y_d.cross(B_z_d);
    R_d_w.col(1) = B_y_d;
    R_d_w.col(2) = B_z_d;

    Eigen::Quaterniond q_temp(R_d_w);
    *desired_quaternion = q_temp;

    // Attitude tracking.
    Eigen::Vector3d tau;

    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_d_w.transpose() * R_B_W_ - R_B_W_.transpose() * R_d_w);
    Eigen::Vector3d e_R;
    e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);

    const Eigen::Vector3d omega_ref =
            r_yaw_rate * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;

    tau = -attitude_gain_.cwiseProduct(e_R)
           - angular_rate_gain_.cwiseProduct(e_omega)
           + angular_velocity_B_.cross(_inertia_matrix.asDiagonal() * angular_velocity_B_);

    // Output the wrench
    *controller_torque_thrust << tau, thrust;

    // Save the data to a file
    saveDataToFile(tau, thrust);
}

void controller::saveDataToFile(const Eigen::Vector3d &tau, double thrust) {
    // Open a file in append mode
    
}
