# Simulation Demo
This file provides the steps needed to run the demo. The steps shown in ``instructions/setup.md`` must be completed first.

> **WARNING:** This demo is not safe for use on real hardware, the circle trajector generator may cause agressive and unpredictable behavior upon takeoff.

## 1. Start the simulation
Navigate to your PX4-Autopilot folder ande execute:
```bash
make px4_sitl_nolockstep gz_x500
```

## 2. Run the MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```

## 3. Launch the controller
```bash
# Source the workspace
source ~/nnc_ws/install/setup.sh

# Start the simulation
ros2 launch px4_offboard_lowlevel x500_exp.launch.py
```

## 4. Start flying
Enabling the trigger will cause the robot to crash, because the robot will anticipate an impulse but none is actually applied.

1. Take off in position mode.
2. Use ``ros2 topic echo /fmu/out/vehicle_odometry`` to monitor the UAVs current position.
3. Send a setpoint at the current position (Flip Y and Z sign), for example:
    ```bash
    ros2 topic pub --once /command/pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 0.0, y: 0.0, z: 10.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    ```
4. Switch to offboard mode in QGroundControl
5. Send the trigger signal:
    ```bash
    ros2 topic pub --once /command/trigger std_msgs/msg/Float32 "{data: 1.0}"
    ```

<!--
### 4.2. Circle trajactory
1. Run the circle trajectory generator in a new terminal
```bash
# Source the workspace
source ~/nnc_ws/install/setup.sh

# Start the trajectory generator
ros2 run px4_offboard_lowlevel circle_trajectory_node
```

2. Open QGroundControl, change to offboard mode, and arm the drone.
-->

<!--
## Change the control mode
By default the controller will be sending [Attitude Setpoints](https://docs.px4.io/main/en/msg_docs/VehicleAttitudeSetpoint.html):
- You can switch to [Thrust](https://docs.px4.io/main/en/msg_docs/VehicleThrustSetpoint.html) + [Torque setpoints](https://docs.px4.io/main/en/msg_docs/VehicleTorqueSetpoint.html) mode by changing `control_mode` parameter:
```bash
ros2 param set /offboard_controller control_mode 2
```

- you can switch to [Direct Actuator commands](https://docs.px4.io/main/en/msg_docs/ActuatorMotors.html) (throttles of the motors) by changing `control_mode` parameter:
```bash
ros2 param set /offboard_controller control_mode 3
```
-->
