clear;clc;
% Define parameters
iris_parameters.zero_position_armed = 100;
iris_parameters.input_scaling = 1000;
iris_parameters.thrust_constant = 5.84e-06;
iris_parameters.moment_constant = 0.06;
iris_parameters.arm_length = 0.25;

% Test PX4-SITL-inverse model
desired_hovering_wrench = [0; 0; 0; 1.725 * 9.81]
[throttles] = func_px4_sitl_inverse_quadrotor_x(desired_hovering_wrench, iris_parameters)
% Test PX4-SITL model
[simulated_wrench] = func_px4_sitl_quadrotor_x(throttles, iris_parameters)

function [throttles, normalized_torque_and_thrust] = func_px4_sitl_inverse_quadrotor_x(wrench, parameters)
% This model represent the control allocation model that inverses PX4
% internal processes

% Parse parameters
zero_position_armed = parameters.zero_position_armed;
input_scaling = parameters.input_scaling;
thrust_constant = parameters.thrust_constant;
moment_constant = parameters.moment_constant;
arm_length = parameters.arm_length;

% Calculate matrices
kS = sin(deg2rad(45));
rotor_velocities_to_torques_and_thrust =    [-kS, kS, kS, -kS;
                                             -kS, kS, -kS, kS;
                                             -1, -1, 1, 1;
                                             1, 1, 1, 1];
helper_matrix = diag([   arm_length *   thrust_constant; ...
                         arm_length *   thrust_constant; ...
                    moment_constant *   thrust_constant; ...
                                        thrust_constant]);
rotor_velocities_to_torques_and_thrust = helper_matrix * rotor_velocities_to_torques_and_thrust;
torques_and_thrust_to_rotor_velocities = pinv(rotor_velocities_to_torques_and_thrust);

mixing_matrix =  [-0.495384,  -0.707107,  -0.765306,   1.0;
                   0.495384, 0.707107,  -1.0,   1.0;
                   0.495384,  -0.707107, 0.765306,   1.0;
                  -0.495384, 0.707107, 1.0,   1.0]; 
throttles_to_normalized_torques_and_thrust = pinv(mixing_matrix);

% From wrench [3D torques; 1D thrust] to rotational velocities [rad/s]
omega_sq = torques_and_thrust_to_rotor_velocities * wrench;
omega = sqrt(omega_sq);
% From rotational velocities to motors throttles based on PX4-SITL models
throttles = (omega - zero_position_armed)./input_scaling;
% Inverse Mixing: throttles to normalized torques and thrust
normalized_torque_and_thrust = throttles_to_normalized_torques_and_thrust * throttles;
end

function [wrench,omega,indv_forces] = func_px4_sitl_quadrotor_x(throttles, parameters)
% This model represents the internal processes of PX4-SITL from receiving
% motors throttles to applying the wrench [3D torques; 1D thrust] to the 
% simulated model

% Parse parameters
zero_position_armed = parameters.zero_position_armed;
input_scaling = parameters.input_scaling;
thrust_constant = parameters.thrust_constant;
moment_constant = parameters.moment_constant;
arm_length = parameters.arm_length;

% Calculate matrices
kS = sin(deg2rad(45));
rotor_velocities_to_torques_and_thrust =    [-kS, kS, kS, -kS;
                                             -kS, kS, -kS, kS;
                                             -1, -1, 1, 1;
                                             1, 1, 1, 1];
helper_matrix = diag([   arm_length *   thrust_constant; ...
                         arm_length *   thrust_constant; ...
                    moment_constant *   thrust_constant; ...
                                        thrust_constant]);
rotor_velocities_to_torques_and_thrust = helper_matrix * rotor_velocities_to_torques_and_thrust;

% PX4-SITL Model
% from throttles to rotational velocities [rad/s] (gazebo_mavlink_interface.cpp)
omega = throttles * input_scaling + zero_position_armed;
% from rotational velocities to force and torques (gazebo_motor_model)
indv_forces = omega .* abs(omega) * thrust_constant; % just for reference
wrench = rotor_velocities_to_torques_and_thrust * omega.^2;
end