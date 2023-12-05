# Package Setup
This setup guide assumes you have installed the Micro XRCE-DDS Agent and PX4 v1.14.0-rc1
as explained in [PX4 and ROS 2 setup](ros_px4_setup.md)
## 1. ROS 2 setup

### 1.1. Create a workspace
This guide assumes the workspace to be in ``~/llc/llc_ws/`` but any desired name/location can be used.
```bash
mkdir -p ~/llc/llc_ws/src
```

### 1.2. Clone the package
```bash
cd ~/llc/llc_ws/src
git clone https://github.com/SaxionMechatronics/px4_offboard_lowlevel.git
```

### 1.3. Clone px4_msgs
This package has been confirmed to work with a certain commit of px4_msgs, you can obtain this commit like so:
```bash
cd ~/llc/llc_ws/src
git clone https://github.com/PX4/px4_msgs.git

cd px4_msgs
# git reset --hard e3d36168a97f0268ab97e626d14858ca644924ef
```

### 1.4. Build
```bash
cd ~/llc/llc_ws/
colcon build --symlink-install
```

## 2. PX4 Setup

### 2.1. Change set of ROS 2 PX4 topics
Before building PX4 you need to change the list of topics for the μXRCE Client, to do this replace the default topics configuration in ```PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml``` by ```px4-resources/dds_topics.yaml``` from this repository.

```bash
cp /PATH_TO/src/px4_offboard_lowlevel_demo/px4-resources/dds_topics.yaml PATH_TO/PX4-Autopilot/src/modules/uxrce_dds_client/
```

### 2.2. Disable lockstep in PX4
Lockstep needs to be disabled in PX4 and Gazebo to make the simulation run correctly while using thrust/torque commands.

Enter the board config for px4 sitl
```bash
make px4_sitl boardconfig
```
Navigate to ``Toolchain`` and enable ``force disable lockstep``

Quit (Q) and save (Y)

Open the gazebo model you will be using for simulation, for example: ``PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf``.

Change enable_lockstep (line 466) from 1 to 0 like so:
```XML
<enable_lockstep>0</enable_lockstep>
```

### 2.4 (optional) Remove the asphalt plane from Gazebo-classic default world

I hate it, so here it is:
```bash
cp /PATH_TO/src/px4_offboard_lowlevel_demo/px4-resources/empty.world PATH_TO/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```