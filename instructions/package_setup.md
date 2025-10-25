# Package Setup
This setup guide assumes you have installed the Micro XRCE-DDS Agent and PX4 v1.16.0
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