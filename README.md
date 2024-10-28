# PX4 Low Level Offboard Control using ROS 2

This package is an example of how to control a [PX4](https://docs.px4.io/main/en/) Multi-rotor Vehicle in [offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode with **Low-level commands** through ROS 2. 

By Low-level commands, we mean that the commands that are sent to PX4 are either:
1. [Attitude Setpoints](https://docs.px4.io/main/en/msg_docs/VehicleAttitudeSetpoint.html) (Collective Thrust + Attitude), 

2. [Thrust](https://docs.px4.io/main/en/msg_docs/VehicleThrustSetpoint.html) + [Torque setpoints](https://docs.px4.io/main/en/msg_docs/VehicleTorqueSetpoint.html)

3. [Direct Actuator commands](https://docs.px4.io/main/en/msg_docs/ActuatorMotors.html) (throttles of the motors). 

You can switch between the different control modes using ROS 2 parameter.

This videos below shows a simulated quadrotor controlled with this package with [Direct Actuator commands](https://docs.px4.io/main/en/msg_docs/ActuatorMotors.html).

![Direct-Actuator-commands](./instructions/media/iris-sitl-act-cmds.gif)

# Contents
## [Package Setup](instructions/package_setup.md)
Guide on the installation of the low level controller package and its dependencies.

## [Explanation](instructions/explanation.md)
Explanation of the controller node and where you can implement your own controller.

## [Simulation Demo](instructions/demo.md)
Instructions to run the demo using the included circle trajector generator.

# Acknowledgments

This work from the SMART research group in Saxion University of Applied Sciences was supported in part by:
* Regioorgaan SIA under project **RAAK-PRO MARS4EARTH** (RAAK.PRO03.112):
* Horizon Europe CSA under project: **AeroSTREAM** (Grant Agreement number: 101071270).

<p align="left">
  <img src="./instructions/media/logos.png" alt="aerostream-logo"/>
</p>

This package was inspired by many open-source package form the PX4 Community, so would like to thank all the contributors of this great community.