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
MicroXRCEAgent udp4 -p 8888; exec bash
ros2 run plotjuggler plotjuggler

```

## 3. Launch the controller
```bash
# Source the workspace
source llc_ws/install/setup.sh

# Start the simulation
ros2 launch px4_offboard_lowlevel iris_sitl.launch.py
# For Visualisation in Rivz
ros2 launch px4_offboard_lowlevel visualize.launch.py
```

## 4. Start flying
1. Run the circle trajector generator in a new terminal
```bash
# Source the workspace
source ~/llc_ws/install/setup.sh
ros2 run px4_offboard_lowlevel circle_trajectory_node

# Start the trajectory generator
```

2. Open QGroundControl, change to offboard mode, and arm the drone.

## Change the controller from SE(3) to Feedback Linearization controller

```bash
ros2 param set /offboard_controller control_mode 2
```
