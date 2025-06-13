# Package Setup
This package uses a custom fork of PX4 for the custom airfame that is used with the included nn controller.

This setup guide assumes you have installed the Micro XRCE-DDS Agent and the fork of PX4-Autopilot as explained in [PX4 and ROS 2 setup](ros_px4_setup.md)

## 1. ROS 2 setup

### 1.1. Create a workspace
This guide assumes the workspace to be in ``~/nnc/nnc_ws/`` but any desired name/location can be used.
```bash
mkdir -p ~/nnc/nnc_ws/src
```

### 1.2. Clone the package
```bash
cd ~/nnc/nnc_ws/src
git clone https://github.com/SaxionMechatronics/px4_offboard_lowlevel.git
git checkout nn-control
```

### 1.3. Clone px4_msgs
This package has been confirmed to work with a certain commit of px4_msgs, you can obtain this commit like so:
```bash
cd ~/nnc/nnc_ws/src
git clone https://github.com/PX4/px4_msgs.git

cd px4_msgs
# git reset --hard 5f8e3c59619666020a546c5264498c6606eca15a
```

### 1.4. Build
```bash
cd ~/nnc/nnc_ws/
colcon build --symlink-install
```

## 2. PX4 Setup

### 2.1. Change set of ROS 2 PX4 topics
Before building PX4 you need to change the list of topics for the μXRCE Client, to do this replace the default topics configuration in ```PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml``` by ```px4-resources/dds_topics.yaml``` from this repository.

```bash
cp /PATH_TO/src/px4_offboard_lowlevel/px4-resources/dds_topics.yaml PATH_TO/PX4-Autopilot/src/modules/uxrce_dds_client/
```

### 2.3 (optional) Remove the asphalt plane from Gazebo-classic default world

I hate it, so here it is:
```bash
cp /PATH_TO/src/px4_offboard_lowlevel/px4-resources/empty.world PATH_TO/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```
