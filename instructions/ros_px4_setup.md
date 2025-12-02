# PX4 and ROS 2 setup

# Install PX4

1. Clone [our fork of the PX4 Source code](https://github.com/SaxionMechatronics/PX4-Autopilot), which contains the custom airframe used for the included nn-controller.

   - Clone into any directory 

```bash
    # for example into ~/nnc
    cd ~/nnc

    # clone
    git clone https://github.com/SaxionMechatronics/PX4-Autopilot.git --recursive

    # checkout to the branch with the custom aiframe
    git checkout nn-control
    git submodule update --recusive
```

2. Run the ubuntu.sh with no arguments (in a bash shell) to install everything

```bash
    cd ~/nnc
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

2. [ROS2 Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
   - Follow this [link](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) to install ROS2
   - Version : Jazzy
   - Ubuntu Version : [Noble Numbat (24.04.1)](https://releases.ubuntu.com/noble/) 

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
    cd ~/nnc

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
    cd ~/nnc/PX4-Autopilot

    # Start a PX4 Gazebo simulation using:
    make px4_sitl_nolockstep gz_x500

    # The agent and client are now running they should connect.
```

# Troubleshooting

1. Unknown target 'gz_rl_frame'
```bash
    # change directory 
    cd ~/nnc/PX4-Autopilot

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
