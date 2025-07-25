# PX4 Neural Offboard Control using ROS 2
This package is an adapted version of the px4_offboard_lowlevel package, this version has implementations to deploy neural network control policies using the ONNX format. A policy for position control of a Holybro X500 is included in this repository as an example.

# Contents
## [Package Setup](instructions/package_setup.md)
Guide on the installation of the nn-control branch of the low level controller package and its dependencies.

## [Explanation](instructions/explanation.md)
Explanation of the controller node and where you can implement your own neural network control policy.

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