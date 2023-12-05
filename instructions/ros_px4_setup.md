# PX4 and ROS 2 setup

# Install PX4

1. [Clone PX4 Source code](https://github.com/PX4/PX4-Autopilot)

   - Clone into any directory 

```bash
    # for example into ~/llc
    cd ~/llc

    # clone
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

   <!-- - If working with older version of PX4-Autopilot, change the head of this clone
   - Head for this package can be found [PX4-Autopilot github (main->Tags)](https://github.com/PX4/PX4-Autopilot/tags)

```bash
    # change directory to PX4
    cd ~/llc/PX4-Autopilot
    # for example
    git checkout v1.14.0-rc1

    # make sure tags are correctly checked out
    git branch

    # update the submodules
    make submodulesclean
``` -->

2. Run the ubuntu.sh with no arguments (in a bash shell) to install everything

```bash
    cd ~/llc
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
3. Restart the system

4. Verify the [NuttX](https://nuttx.apache.org/) installation
```bash
    arm-none-eabi-gcc --version

    # The following output will be shown 
    arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
    Copyright (C) 2019 Free Software Foundation, Inc.
    This is free software; see the source for copying conditions.  There is NO
    warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

# ROS2 With PX4
For more information on ROS 2 refer to the [ROS 2 Documentation](https://docs.ros.org/en/humble/)

1. [Install PX4](#install-px4) 
   - As explained above

2. [ROS2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - Follow this [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS2
   - Version : Humble
   - Ubuntu Version : [Jammy Jellyfish (22.04)](https://releases.ubuntu.com/jammy/) 

3. Install Python dependencies 
```bash
    pip install --user -U empy pyros-genmsg setuptools

    # if pip is not installed then:
    sudo apt update
    
    # Install pip for Python 3
    sudo apt install python3-pip
    
    # verify the install
    pip3 --version

    # upgrade pip3 to the latest version
    sudo pip3 install --upgrade pip
```

4. Setup Micro XRCE-DDS Agent & Client
```bash
    # change the diectory
    cd ~/llc

    # Fetch and build the agent
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
```
5. Start the agent
```bash
    # start the micro xrcedds agent in a new terminal window using this command line below
    MicroXRCEAgent udp4 -p 8888
```

6. Start the simulation
```bash
    # change directory to PX4
    cd ~/llc/PX4-Autopilot

    # Start a PX4 Gazebo simulation using:
    make px4_sitl gz_x500

    # The agent and client are now running they should connect.
```

# Troubleshooting

1. Unknown target 'gz_x500'
```bash
    # change directory 
    cd ~/llc/PX4-Autopilot

    # run this command
    make clean
    make distclean
```

# [QGroundControl](http://qgroundcontrol.com/)

1. Why Its Needed ?

QGroundControl is essential software for managing and controlling unmanned aerial vehicles (UAVs) and autonomous systems, offering mission planning, real-time monitoring, manual control, data visualization, firmware updates, logging, telemetry, and customization capabilities, facilitating safe and efficient operation of these vehicles.

2. [Installation](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

   - Before installing QGroundControl for the first time:

```bash
    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libqt5gui5 -y
    sudo apt install libfuse2 -y
```
   - Download QGroundControl image (from steps 2)
   - Probably downloaded image will be in /Download directory

```bash
    cd 
    cd Download

    chmod +x ./QGroundControl.AppImage
    # run the image
    ./QGroundControl.AppImage
```
