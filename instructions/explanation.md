# Steps for implementing your own control policy (thrust + rates)
For simplicity these steps assume you are modifying the x500 configuration files, but you may also create a new launch file and configs.

1. Place the onnx file containing the desired control policy in the `policy` folder.
2. Set the onnx filename, and the thrust and rate scale for your policy.
3. Adapt the `getObs` function in [`controller.cpp`](/src/controller.cpp) to construct the observation vector as required by your policy.
4. In the same file, adapt the post-processing of the policy output (clipping, scaling, gravity compensation, yaw control, etc.) as desired.

# Code explanation
The ``px4_offboard_lowlevel`` node of this package consists of [``controller_node.cpp``](/src/controller_node.cpp) for the ControllerNode class which handles communication to ROS2 and PX4, and [``controller.cpp``](/src/controller.cpp) which contains the controller itself, both have corresponding header files in [``include/px4_offboard_lowlevel``](/include/px4_offboard_lowlevel).

In this fork [``controller.cpp``](/src/controller.cpp) contains the neural network implementation. The network is loaded from a file in the [ONNX](https://onnx.ai/) format.

## Overview
![Schematic overview](media/LowLevelOffboard.png)

## ControllerNode
The ControllerNode class implements the interface between the controller, ROS2 and PX4. It provides the controller with odometry and setpoints, and it requests the controller to calculate desired output at a set interval.

## Controller
The desired control output is requested by the ControllerNode using the `calculateControllerOutput` function in [``controller.cpp``](/src/controller.cpp). In there the observation vector is constructed using the `getObs` function, which is then forwarded through the neural network using `forwardPolicy`. The output is then processed for the correct scaling and gravity compensation, and the desired yaw rate is calculated using a simple proportional controller.

In the current implementation controller_rate_thrust is used to set the desired thrust and rate, this can be changed to another control mode in [``controller_node.cpp``](/src/controller_node.cpp).

The onnx file containing the nn-controller should be placed in the `policy` folder. The filename can be provided as a ros parameter `policy.filename`, for the provided examples this is done in the policy configuration files.
