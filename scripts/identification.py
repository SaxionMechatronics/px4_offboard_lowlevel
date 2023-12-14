#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from os.path import isfile

import rclpy
from rclpy.node import Node

def quadratic(x, a):
    return(a*x**2)

def linear(x, a):
    return(a*x)

def polynomial(x,a,b,c):
    return(a*x**2 + b*x + c)

def main(args=None):
    rclpy.init(args=args)
    node = Node("identification")

    node.declare_parameter("file_in","input.csv")
    node.declare_parameter("file_out","uav_params.yaml")

    # Read log
    name_of_log_file = node.get_parameter("file_in").value

    if not isfile(name_of_log_file):
        node.get_logger().error("File '" + name_of_log_file + "' does not exist")
        return
    
    log_data = pd.read_csv(name_of_log_file, sep=",", header=0)

    times = log_data["Time (s)"]

    ptName = log_data.keys()[1][:log_data.keys()[1].find('-') - 1]

    print('Selected powertrain: "' + ptName + '"')

    ESCSignals = log_data[ptName + " - ESC throttle (μs)"]
    TorqueNm = log_data[ptName + " - torque MZ (torque) (N⋅m)"]
    ThrustN = log_data[ptName + " - force Fz (thrust) (N)"]

    if ptName + " - rotation speed (rad/s)" in log_data:
        MotorElectricalSpeedrads = log_data[ptName + " - rotation speed (rad/s)"]
    else:
        MotorElectricalSpeedrads = log_data[ptName + " - rotation speed (rpm)"]  / 60 * 2 * np.pi;

    # Flip thrust and torque
    if sum(ThrustN) < 0:
        ThrustN = -ThrustN

    if sum(TorqueNm) < 0:
        TorqueNm = -TorqueNm

    # Ranges for model plotting
    upperRads = round(max(MotorElectricalSpeedrads) * 1.1)
    rads = range(0,upperRads)

    upperThrust = round(max(ThrustN) * 1.1)
    thrust = range(0,upperThrust)

    # Thrust constant estimation
    thrust_mdl = curve_fit(quadratic, MotorElectricalSpeedrads, ThrustN)
    k_thrust = thrust_mdl[0][0]

    node.get_logger().info("k_thrust: " + str(k_thrust))
    thrust_model = k_thrust * np.square(rads)

    plt.plot(MotorElectricalSpeedrads, ThrustN, "-o", label="Measured")
    plt.plot(rads, thrust_model, label="Model")
    plt.legend()
    plt.xlabel("Motor velocity [rad/s]")
    plt.ylabel("Thrust [N]")
    plt.show()

    # Torque constant estimation
    torque_mdl = curve_fit(quadratic, MotorElectricalSpeedrads, TorqueNm)
    k_torque = torque_mdl[0][0]
    
    node.get_logger().info("k_torque: " + str(k_torque))

    torque_model = k_torque * np.square(rads)

    plt.plot(MotorElectricalSpeedrads, TorqueNm, "-o", label="Measured")
    plt.plot(rads, torque_model, label="Model")
    plt.legend()
    plt.xlabel("Motor velocity [rad/s]")
    plt.ylabel("Torque [Nm]")
    plt.show()

    # Torque coefficient estimation
    torque_coefficient_mdl = curve_fit(linear, ThrustN, TorqueNm)
    torque_coefficent = torque_coefficient_mdl[0][0]

    node.get_logger().info("torque_coefficent: " + str(torque_coefficent))

    torque_coefficent_model = torque_coefficent * thrust

    plt.plot(ThrustN, TorqueNm, "-o", label="Measured")
    plt.plot(thrust, torque_coefficent_model, label="Model")
    plt.legend()
    plt.xlabel("Thrust [N]")
    plt.ylabel("Torque [Nm]")
    plt.show()

    # omega_to_pwm_coeffients
    omega_to_pwm_mdl = curve_fit(polynomial, MotorElectricalSpeedrads, ESCSignals)

    p1 = omega_to_pwm_mdl[0][0]
    p2 = omega_to_pwm_mdl[0][1]
    p3 = omega_to_pwm_mdl[0][2]

    node.get_logger().info("p1: " +  str(p1))
    node.get_logger().info("p2: " +  str(p2))
    node.get_logger().info("p2: " +  str(p3))

    ESCs = p1 * np.square(rads) + p2 * rads + p3

    plt.plot(MotorElectricalSpeedrads, ESCSignals, "-o", label="Measured")
    plt.plot(rads, ESCs, label="Model")
    plt.legend()
    plt.xlabel("Motor velocity [rad/s]")
    plt.ylabel("PWM [\u03BCs]")
    plt.show()

    # write to file
    file_out_name = node.get_parameter("file_out").value
    f = open(file_out_name,"w")

    mass = "" #1.574
    arm_length = "" #0.25
    num_of_arms = "" #4
    inertia = ["", "", ""] #[0.08612, 0.08962, 0.16088]
    gravity = 9.81
    pwm_min = 1100
    pwm_max = 1900
    input_scaling = 1000
    zero_position_armed = 10

    f.write("""/**:
    ros__parameters:
        uav_parameters:\n""")
    f.write("          mass: " + str(mass) + "\n")
    f.write("          arm_length: " + str(arm_length) + "\n")
    f.write("          num_of_arms: " + str(num_of_arms) + "\n")
    f.write("          inertia:\n")
    f.write("            x: " + str(inertia[0]) + "\n")
    f.write("            y: " + str(inertia[1]) + "\n")
    f.write("            z: " + str(inertia[2]) + "\n")
    f.write("          moment_constant: " + str(torque_coefficent) + "\n")
    f.write("          thrust_constant: " + str(k_thrust) + "\n")
    f.write("          max_rotor_speed: " + str(round(np.max(MotorElectricalSpeedrads))) + "\n")
    f.write("          gravity: " + str(gravity) + "\n")
    f.write("          omega_to_pwm_coefficient:\n")
    f.write("            x_2: " + str(p1) + "\n")
    f.write("            x_1: " + str(p2) + "\n")
    f.write("            x_0: " + str(p3) + "\n")
    f.write("          PWM_MIN: " + str(pwm_min) + "\n")
    f.write("          PWM_MAX: " + str(pwm_max) + "\n")
    f.write("          input_scaling: " + str(input_scaling) + "\n")
    f.write("          zero_position_armed: " + str(zero_position_armed) + "\n")

    f.close()

    node.get_logger().info("Results written to: " + file_out_name)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
