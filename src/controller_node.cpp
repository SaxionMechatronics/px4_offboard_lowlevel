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

#include "px4_offboard_lowlevel/controller_node.h"



ControllerNode::ControllerNode() 
    : Node("controller_node")
    {
        loadParams();
        compute_ControlAllocation_and_ActuatorEffect_matrices();

        // Defining the compatible ROS 2 predefined QoS for PX4 topics
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Subscribers
        vehicle_odometry_sub_= this->create_subscription<px4_msgs::msg::VehicleOdometry>
            (odometry_topic_, qos, std::bind(&ControllerNode::vehicle_odometryCallback, this, _1));
        vehicle_status_sub_= this->create_subscription<px4_msgs::msg::VehicleStatus>
            (status_topic_, qos, std::bind(&ControllerNode::vehicleStatusCallback, this, _1));
        command_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (command_pose_topic_, 10, std::bind(&ControllerNode::commandPoseCallback, this, _1));

        // Publishers
        attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>
            (attitude_setpoint_topic_, 10);  
        actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>
            (actuator_control_topic_, 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>
            (offboard_control_topic_, 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>
            (vehicle_command_topic_, 10);
        thrust_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>
            (thrust_setpoint_topic_, 10);
        torque_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>
            (torque_setpoint_topic_, 10);
        
        // Parameters subscriber
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ControllerNode::parametersCallback, this, std::placeholders::_1));

        // Timers
        std::chrono::duration<double> offboard_period(0.33);        
        std::chrono::duration<double> controller_period(0.01);        
        offboardTimer = this->create_wall_timer(offboard_period, [=]() {publishOffboardControlModeMsg();});
        controllerTimer = this->create_wall_timer(controller_period, [=]() {updateControllerOutput();});
    }

rcl_interfaces::msg::SetParametersResult ControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        // print info about the changed parameter
        for (const auto &param: parameters){
            RCLCPP_INFO(this->get_logger(), "Parameter %s has changed to [%s]", param.get_name().c_str(), param.value_to_string().c_str());
            if(param.get_name() == "control_mode"){
                control_mode_ = param.as_int();
            }
        }
        return result;
    }

void ControllerNode::loadParams() {
    // UAV Parameters
    this->declare_parameter("uav_parameters.mass", 0.0);
    this->declare_parameter("uav_parameters.arm_length", 0.0);
    this->declare_parameter("uav_parameters.num_of_arms", 4);
    this->declare_parameter("uav_parameters.moment_constant", 0.0);
    this->declare_parameter("uav_parameters.thrust_constant", 0.0);
    this->declare_parameter("uav_parameters.max_rotor_speed", 0);
    this->declare_parameter("uav_parameters.gravity", 0.0);
    this->declare_parameter("uav_parameters.PWM_MIN", 0);
    this->declare_parameter("uav_parameters.PWM_MAX", 0);
    this->declare_parameter("uav_parameters.input_scaling", 0);
    this->declare_parameter("uav_parameters.zero_position_armed", 0);
    this->declare_parameter("uav_parameters.inertia.x", 0.0);
    this->declare_parameter("uav_parameters.inertia.y", 0.0);
    this->declare_parameter("uav_parameters.inertia.z", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_2", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_1", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_0", 0.0);

    double _uav_mass = this->get_parameter("uav_parameters.mass").as_double();
    _arm_length = this->get_parameter("uav_parameters.arm_length").as_double();
    _num_of_arms = this->get_parameter("uav_parameters.num_of_arms").as_int();
    _moment_constant = this->get_parameter("uav_parameters.moment_constant").as_double();
    _thrust_constant = this->get_parameter("uav_parameters.thrust_constant").as_double();
    _max_rotor_speed = this->get_parameter("uav_parameters.max_rotor_speed").as_int();
    double _gravity = this->get_parameter("uav_parameters.gravity").as_double();
    _PWM_MIN = this->get_parameter("uav_parameters.PWM_MIN").as_int();
    _PWM_MAX = this->get_parameter("uav_parameters.PWM_MAX").as_int();
    _input_scaling = this->get_parameter("uav_parameters.input_scaling").as_int();
    _zero_position_armed = this->get_parameter("uav_parameters.zero_position_armed").as_int();
    double _inertia_x = this->get_parameter("uav_parameters.inertia.x").as_double();
    double _inertia_y = this->get_parameter("uav_parameters.inertia.y").as_double();
    double _inertia_z = this->get_parameter("uav_parameters.inertia.z").as_double();
    double _omega_to_pwm_coefficient_x_2 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_2").as_double();
    double _omega_to_pwm_coefficient_x_1 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_1").as_double();
    double _omega_to_pwm_coefficient_x_0 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_0").as_double();
    Eigen::Vector3d _inertia_matrix;
    _inertia_matrix << _inertia_x, _inertia_y, _inertia_z;
    _omega_to_pwm_coefficients << _omega_to_pwm_coefficient_x_2, _omega_to_pwm_coefficient_x_1, _omega_to_pwm_coefficient_x_0;
    
    // Topics Names
    this->declare_parameter("topics_names.command_pose_topic", "default");
    this->declare_parameter("topics_names.command_traj_topic", "default");
    this->declare_parameter("topics_names.odometry_topic", "default");
    this->declare_parameter("topics_names.status_topic", "default");
    this->declare_parameter("topics_names.battery_status_topic", "default");
    this->declare_parameter("topics_names.actuator_status_topic", "default");
    this->declare_parameter("topics_names.offboard_control_topic", "default");
    this->declare_parameter("topics_names.vehicle_command_topic", "default");
    this->declare_parameter("topics_names.attitude_setpoint_topic", "default");
    this->declare_parameter("topics_names.thrust_setpoints_topic", "default");
    this->declare_parameter("topics_names.torque_setpoints_topic", "default");
    this->declare_parameter("topics_names.actuator_control_topic", "default");

    command_pose_topic_ = this->get_parameter("topics_names.command_pose_topic").as_string();
    command_traj_topic_ = this->get_parameter("topics_names.command_traj_topic").as_string();
    odometry_topic_ = this->get_parameter("topics_names.odometry_topic").as_string();
    status_topic_ = this->get_parameter("topics_names.status_topic").as_string();
    battery_status_topic_ = this->get_parameter("topics_names.battery_status_topic").as_string();
    actuator_status_topic = this->get_parameter("topics_names.actuator_status_topic").as_string();
    offboard_control_topic_ = this->get_parameter("topics_names.offboard_control_topic").as_string();
    vehicle_command_topic_ = this->get_parameter("topics_names.vehicle_command_topic").as_string();
    attitude_setpoint_topic_ = this->get_parameter("topics_names.attitude_setpoint_topic").as_string();
    thrust_setpoint_topic_ = this->get_parameter("topics_names.thrust_setpoints_topic").as_string();
    torque_setpoint_topic_ = this->get_parameter("topics_names.torque_setpoints_topic").as_string();
    actuator_control_topic_ = this->get_parameter("topics_names.actuator_control_topic").as_string();
    
    // Load logic switches
    this->declare_parameter("sitl_mode", true);
    this->declare_parameter("control_mode", 1);

    in_sitl_mode_ = this->get_parameter("sitl_mode").as_bool();
    control_mode_ = this->get_parameter("control_mode").as_int();
    
    // Controller gains
    this->declare_parameter("control_gains.K_p_x", 0.0);
    this->declare_parameter("control_gains.K_p_y", 0.0);
    this->declare_parameter("control_gains.K_p_z", 0.0);
    this->declare_parameter("control_gains.K_v_x", 0.0);
    this->declare_parameter("control_gains.K_v_y", 0.0);
    this->declare_parameter("control_gains.K_v_z", 0.0);
    this->declare_parameter("control_gains.K_R_x", 0.0);
    this->declare_parameter("control_gains.K_R_y", 0.0);
    this->declare_parameter("control_gains.K_R_z", 0.0);
    this->declare_parameter("control_gains.K_w_x", 0.0);
    this->declare_parameter("control_gains.K_w_y", 0.0);
    this->declare_parameter("control_gains.K_w_z", 0.0);
    
    position_gain_ << this->get_parameter("control_gains.K_p_x").as_double(),
                      this->get_parameter("control_gains.K_p_y").as_double(),
                      this->get_parameter("control_gains.K_p_z").as_double();

    velocity_gain_ << this->get_parameter("control_gains.K_v_x").as_double(),
                      this->get_parameter("control_gains.K_v_y").as_double(),
                      this->get_parameter("control_gains.K_v_z").as_double();

    attitude_gain_ << this->get_parameter("control_gains.K_R_x").as_double(),
                      this->get_parameter("control_gains.K_R_y").as_double(),
                      this->get_parameter("control_gains.K_R_z").as_double();

    ang_vel_gain_ << this->get_parameter("control_gains.K_w_x").as_double(),
                     this->get_parameter("control_gains.K_w_y").as_double(),
                     this->get_parameter("control_gains.K_w_z").as_double();

    // pass the UAV Parameters and controller gains to the controller
    controller_.setUavMass(_uav_mass);
    controller_.setInertiaMatrix(_inertia_matrix);
    controller_.setGravity(_gravity);
    controller_.setKPositionGain(position_gain_);
    controller_.setKVelocityGain(velocity_gain_);
    controller_.setKAttitudeGain(attitude_gain_);
    controller_.setKAngularRateGain(ang_vel_gain_);
}

void ControllerNode::compute_ControlAllocation_and_ActuatorEffect_matrices() {
    const double kDegToRad = M_PI / 180.0;
    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust;
    Eigen::MatrixXd mixing_matrix;
    if (_num_of_arms == 4){
        const double kS = std::sin(45 * kDegToRad);
        rotor_velocities_to_torques_and_thrust.resize(4, 4);
        mixing_matrix.resize(4,4);
        rotor_velocities_to_torques_and_thrust <<    -kS, kS, kS, -kS,
                -kS, kS, -kS, kS,
                -1, -1, 1, 1,
                1, 1, 1, 1;
        mixing_matrix <<   -0.495384,  -0.707107,  -0.765306,   1.0,
                 0.495384, 0.707107,  -1.0,   1.0,
                0.495384,  -0.707107, 0.765306,   1.0,
                -0.495384, 0.707107, 1.0,   1.0;
        torques_and_thrust_to_rotor_velocities_.resize(4, 4);
        throttles_to_normalized_torques_and_thrust_.resize(4,4);
        // Hardcoded because the calculation of pesudo-inverse is not accurate
        throttles_to_normalized_torques_and_thrust_ << 
        -0.5718,    0.4376,    0.5718,   -0.4376,
        -0.3536,    0.3536,   -0.3536,    0.3536,
        -0.2832 ,   -0.2832 ,  0.2832 ,  0.2832,
        0.2500 ,   0.2500 ,   0.2500 ,   0.2500;
    }
    else {
        std::cout<<("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }
    // Calculate Control allocation matrix: Wrench to Rotational velocities
    Eigen::Vector4d k;  // Helper diagonal matrix.
    k <<    _thrust_constant * _arm_length, 
            _thrust_constant * _arm_length,
            _moment_constant * _thrust_constant, 
            _thrust_constant;
    rotor_velocities_to_torques_and_thrust = k.asDiagonal() * rotor_velocities_to_torques_and_thrust;
    std::cout << "rotor_velocities_to_torques_and_thrust = " << rotor_velocities_to_torques_and_thrust << std::endl;
    torques_and_thrust_to_rotor_velocities_.setZero();
    torques_and_thrust_to_rotor_velocities_ =
            rotor_velocities_to_torques_and_thrust.completeOrthogonalDecomposition().pseudoInverse();
    std::cout << "rotor_velocities_to_torques_and_thrust = " << rotor_velocities_to_torques_and_thrust << std::endl;
    std::cout << "torques_and_thrust_to_rotor_velocities = " << torques_and_thrust_to_rotor_velocities_ << std::endl;
    std::cout << "throttles_to_normalized_torques_and_thrust_ = " << throttles_to_normalized_torques_and_thrust_ << std::endl;
}

void ControllerNode::px4Inverse
    (Eigen::Vector4d *normalized_torque_and_thrust, Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench) {
    Eigen::VectorXd omega;
    Eigen::VectorXd pwm;
    Eigen::VectorXd ones_temp;
    normalized_torque_and_thrust->setZero();
    if (_num_of_arms == 4){
        omega.resize(4);
        omega.setZero();
        pwm.resize(4);
        pwm.setZero();
        throttles->resize(4);
        throttles->setZero();
        ones_temp.resize(4);
        ones_temp = Eigen::VectorXd::Ones(4,1);
    }
    else {
        std::cout<<("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }
    // Control allocation: Wrench to Rotational velocities (omega)
    omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);
    for (int i = 0; i < omega.size(); i++){
        if (omega[i] <= 0){
            omega[i] = 0.0;
        }
    }
    omega = omega.cwiseSqrt();
    pwm = (_omega_to_pwm_coefficients(0) * omega.cwiseProduct(omega)) + (_omega_to_pwm_coefficients(1) * omega) +
          (_omega_to_pwm_coefficients(2) * ones_temp);
    *throttles = (pwm - (_PWM_MIN * ones_temp));
    *throttles /= (_PWM_MAX - _PWM_MIN);
    // Inverse Mixing: throttles to normalized torques and thrust
    *normalized_torque_and_thrust = throttles_to_normalized_torques_and_thrust_ * *throttles;
}

void ControllerNode::px4InverseSITL
    (Eigen::Vector4d *normalized_torque_and_thrust, Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench) {
    Eigen::VectorXd omega;
    normalized_torque_and_thrust->setZero();
    Eigen::VectorXd ones_temp;
    if (_num_of_arms == 6){
        omega.resize(6);
        omega.setZero();
        throttles->resize(6);
        throttles->setZero();
        ones_temp.resize(6);
        ones_temp = Eigen::VectorXd::Ones(6,1);
    }
    else if (_num_of_arms == 4){
        omega.resize(4);
        omega.setZero();
        throttles->resize(4);
        throttles->setZero();
        ones_temp.resize(4);
        ones_temp = Eigen::VectorXd::Ones(4,1);
    }
    else if (_num_of_arms == 44){
        omega.resize(8);
        omega.setZero();
        throttles->resize(8);
        throttles->setZero();
        ones_temp.resize(8);
        ones_temp = Eigen::VectorXd::Ones(8,1);
    }
    else {
        std::cout<<("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }
    // Control allocation: Wrench to Rotational velocities (omega)
    omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);
    omega = omega.cwiseSqrt();
    *throttles = (omega - (_zero_position_armed * ones_temp));
    *throttles /= (_input_scaling);
    // Inverse Mixing: throttles to normalized torques and thrust
    *normalized_torque_and_thrust = throttles_to_normalized_torques_and_thrust_ * *throttles;
}

void ControllerNode::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void ControllerNode::disarm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void ControllerNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	// vehicle_command_publisher_->publish(msg);
}

void ControllerNode::publishOffboardControlModeMsg()
{
	px4_msgs::msg::OffboardControlMode offboard_msg{};
	offboard_msg.position = false;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.body_rate = false;
    switch (control_mode_)
    {
    case 1:
        offboard_msg.attitude = true;
        offboard_msg.thrust_and_torque = false;
        offboard_msg.direct_actuator = false;
        break;
    case 2:
        offboard_msg.attitude = false;
        offboard_msg.thrust_and_torque = true;
        offboard_msg.direct_actuator = false;
        break;
    case 3:
        offboard_msg.attitude = false;
        offboard_msg.thrust_and_torque = false;
        offboard_msg.direct_actuator = true;
        break;
    default:
        offboard_msg.attitude = true;
        offboard_msg.thrust_and_torque = false;
        offboard_msg.direct_actuator = false;
        break;
    }
	offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(offboard_msg);
    RCLCPP_INFO_ONCE(get_logger(),"Offboard enabled");
}

void ControllerNode::commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {                   // When a command is received
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    eigenTrajectoryPointFromPoseMsg(pose_msg, position, orientation);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
    controller_.setTrajectoryPoint(position, orientation);          // Send the command to controller_ obj
}

void ControllerNode::commandTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr& traj_msg) {                   // When a command is received
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Vector3d velocity; 
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acceleration;
    eigenTrajectoryPointFromMsg( traj_msg, position, orientation, velocity, angular_velocity, acceleration);
    controller_.setTrajectoryPoint(position, velocity, acceleration, orientation, angular_velocity);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
}

void ControllerNode::vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg){
        //  Debug message
        RCLCPP_INFO_ONCE(get_logger(),"Controller got first odometry message.");

        Eigen::Vector3d position;
        Eigen::Vector3d velocity; 
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
        
        eigenOdometryFromPX4Msg(odom_msg,
                                position, orientation, velocity, angular_velocity);

        controller_.setOdometry(position, orientation, velocity, angular_velocity);
}

void ControllerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg){
    current_status_ = *status_msg;
    if (current_status_.arming_state ==2){
        RCLCPP_INFO_ONCE(get_logger(),"ARMED - vehicle_status_msg.");
    }
    else {
        RCLCPP_INFO(get_logger(),"NOT ARMED - vehicle_status_msg.");
    }
    if (current_status_.nav_state == 14){
        RCLCPP_INFO_ONCE(get_logger(),"OFFBOARD - vehicle_status_msg.");
    }
    else {
        RCLCPP_INFO(get_logger(),"NOT OFFBOARD - vehicle_status_msg.");
    }
}

void ControllerNode::publishActuatorMotorsMsg(const Eigen::VectorXd& throttles) {
    // Lockstep should be disabled from PX4 and from the model.sdf file
    // direct motor throttles control
    // Prepare msg
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;
    actuator_motors_msg.control = { (float) throttles[0], (float) throttles[1], (float) throttles[2], (float) throttles[3], 
                            std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                            std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};
	actuator_motors_msg.reversible_flags = 0;
	actuator_motors_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
	actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

	actuator_motors_publisher_->publish(actuator_motors_msg);
}

void ControllerNode::publishThrustTorqueMsg(const Eigen::Vector4d& controller_output) {
    // Lockstep should be disabled from PX4 and from the model.sdf file
    // Prepare msgs
    px4_msgs::msg::VehicleThrustSetpoint thrust_sp_msg;
    px4_msgs::msg::VehicleTorqueSetpoint torque_sp_msg;
    thrust_sp_msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    torque_sp_msg.timestamp_sample = thrust_sp_msg.timestamp_sample ;
    thrust_sp_msg.timestamp = thrust_sp_msg.timestamp_sample ;
    torque_sp_msg.timestamp = thrust_sp_msg.timestamp_sample ;
    // Fill thrust setpoint msg
    thrust_sp_msg.xyz[0] = 0.0;
    thrust_sp_msg.xyz[1] = 0.0;
    if (controller_output[3] > 0.1){
        thrust_sp_msg.xyz[2] = -controller_output[3];         // DO NOT FORGET THE MINUS SIGN (body NED frame)
    }
    else {
        thrust_sp_msg.xyz[2] = -0.1;
    }
    // Rotate torque setpoints from FLU to FRD and fill the msg
    Eigen::Vector3d rotated_torque_sp;
    rotated_torque_sp = rotateVectorFromToFRD_FLU(Eigen::Vector3d(controller_output[0], controller_output[1], controller_output[2]));
    torque_sp_msg.xyz[0] = rotated_torque_sp[0];
    torque_sp_msg.xyz[1] = rotated_torque_sp[1];
    torque_sp_msg.xyz[2] = rotated_torque_sp[2];

    // Publish msgs
    thrust_setpoint_publisher_->publish(thrust_sp_msg);
    torque_setpoint_publisher_->publish(torque_sp_msg);
}

void ControllerNode::publishAttitudeSetpointMsg(const Eigen::Vector4d& controller_output, const Eigen::Quaterniond& desired_quaternion) {    
    // Prepare AttitudeSetpoint msg;
    attitude_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    Eigen::Quaterniond rotated_quat;
    rotated_quat = rotateQuaternionFromToENU_NED(desired_quaternion);
    attitude_setpoint_msg.q_d[0] = rotated_quat.w();
    attitude_setpoint_msg.q_d[1] = rotated_quat.x();
    attitude_setpoint_msg.q_d[2] = rotated_quat.y();
    attitude_setpoint_msg.q_d[3] = rotated_quat.z();

    if (controller_output[3] > 0.1){
        attitude_setpoint_msg.thrust_body[0] = 0.0;
        attitude_setpoint_msg.thrust_body[1] = 0.0;
        attitude_setpoint_msg.thrust_body[2] = -controller_output[3];         // DO NOT FORGET THE MINUS SIGN (body NED frame)
    }
    else {
        attitude_setpoint_msg.thrust_body[2] = -0.1;
    }

    attitude_setpoint_publisher_->publish(attitude_setpoint_msg);
}

void ControllerNode::updateControllerOutput() {
    //  calculate controller output
    Eigen::VectorXd controller_output;
    Eigen::Quaterniond desired_quaternion;
    controller_.calculateControllerOutput(&controller_output, &desired_quaternion);
    
    // Normalize the controller output
    Eigen::Vector4d normalized_torque_thrust;
    Eigen::VectorXd throttles;
    if (in_sitl_mode_) px4InverseSITL(&normalized_torque_thrust, &throttles, &controller_output);
    else px4Inverse(&normalized_torque_thrust, &throttles, &controller_output);
    
    // Publish the controller output
    if (current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        switch (control_mode_)
        {
        case 1:
            publishAttitudeSetpointMsg(normalized_torque_thrust, desired_quaternion);
            break;
        case 2:
            publishThrustTorqueMsg(normalized_torque_thrust);
            break;
        case 3:
            publishActuatorMotorsMsg(throttles);
            break;
        default:
            publishAttitudeSetpointMsg(normalized_torque_thrust, desired_quaternion);
            break;
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}