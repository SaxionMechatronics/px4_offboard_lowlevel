# Simulation Demo
This file provides the steps needed to run the demo. The steps shown in ``instructions/setup.md`` must be completed first.

> **WARNING:** This demo is not safe for use on real hardware, the circle trajector generator may cause agressive and unpredictable behavior upon takeoff.

## 1. Start the simulation
Navigate to your PX4-Autopilot folder ande execute:
```bash
make px4_sitl gazebo-classic_iris
```

## 2. Run the MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```

## 3. Launch the controller
```bash
# Source the workspace
source ~/llc_ws/install/setup.sh

# Start the simulation
ros2 launch px4_offboard_lowlevel iris_sitl.launch.py
```

## 4. Start flying
1. Run the circle trajector generator in a new terminal
```bash
# Source the workspace
source ~/llc_ws/install/setup.sh

# Start the trajectory generator
ros2 run px4_offboard_lowlevel circle_trajectory_node
```

2. Open QGroundControl, change to offboard mode, and arm the drone.

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