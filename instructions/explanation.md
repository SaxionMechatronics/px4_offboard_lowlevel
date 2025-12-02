# Explanation
The ``px4_offboard_lowlevel`` node of this package consists of [``controller_node.cpp``](/src/controller_node.cpp) for the ControllerNode class which handles communication to ROS2 and PX4, and [``controller.cpp``](/src/controller.cpp) which contains the controller itself, both have corresponding header files in [``include/px4_offboard_lowlevel``](/include/px4_offboard_lowlevel). For the implementation of your own controller you will probably only need to make changes to [``controller_node.cpp``](/src/controller_node.cpp).

In this fork [``controller.cpp``](/src/controller.cpp) contains the neural network implementation. In there the network is loaded from a file in the [ONNX](https://onnx.ai/) format, the observation vector is constructed and the policy output is calculated.

# Overview
![Schematic overview](media/LowLevelOffboard.png)

# ControllerNode
The ControllerNode class implements the interface between the controller, ROS2 and PX4. It provides the controller with odometry and setpoints, and it requests the controller to calculate desired output at a set interval.

# Controller
The Controller class implements the controller itself, this is where the neural network control policy is implemented.

## Control policy implementation.
The desired control output is requested by the ControllerNode using the `calculateControllerOutput` function. This is where you can implement your own control policy.

In the current implementation the controller_rate_thrust is used to set the desired thrust and rate, this can be changed to another control mode in [``controller_node.cpp``](/src/controller_node.cpp). 

The `model_path` variable defines the location of the onnx model that is used, by default this is `policy/controller.onnx`.

<!--## Other interfaces
### UAV Parameters
UAV Parameters are passed to the controller using the following functions:
```CPP
setUavMass(double uavMass)
setInertiaMatrix(const Eigen::Vector3d &inertiaMatrix)
setGravity(double gravity)
setKPositionGain(const Eigen::Vector3d &PositionGain)
setKVelocityGain(const Eigen::Vector3d &VelocityGain)
setKAttitudeGain(const Eigen::Vector3d &AttitudeGain)
setKAngularRateGain(Eigen::Vector3d AngularRateGain)
```

### Odometry
Odometry (position, orientation, velocity and angular velocity) is provided to the controller through the setOdometry funciton, this is called by the ControllerNode class every time odometry is received from PX4. 

```CPP
const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_W)
```

### Setpoints
Position and orientation setpoints are provided to the controller through the setTrajectoryPoint function.

```CPP
setTrajectoryPoint(const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_W)
```

## Working with another vehicles

Check the parameters that needs to be defined for each vehicle in `./config/uav_parameters/iris_param.yaml`. By creating another parameters yaml file and another launch file based on `./launch/iris_sitl.launch.py`, you should be able to fly another vehicle with the package.-->
