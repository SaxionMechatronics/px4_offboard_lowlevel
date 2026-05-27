FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8

# Setup
RUN apt-get update && apt-get install -y locales tzdata curl gnupg2 lsb-release \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 related
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list

# Gazebo related
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# Install
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    gz-harmonic \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-empy \
    build-essential cmake git ninja-build ccache \
    libeigen3-dev \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-plotjuggler-ros \
    libssl-dev libxml2-utils \
    astyle python3-toml python3-numpy python3-dev python3-setuptools wget \
    x11-apps \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install empy==3.3.4 pyros-genmsg packaging jinja2 jsonschema kconfiglib

# ROS workspace
RUN mkdir -p /opt/ros/px4_ws/src \
    && cd /opt/ros/px4_ws/src \
    && git clone https://github.com/PX4/px4_msgs.git --branch v1.16.0 --depth 1 \
    && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        --branch v2.4.2 --depth 1 \
    && /bin/bash -c "\
        source /opt/ros/humble/setup.bash && \
        cd /opt/ros/px4_ws && \
        rosdep update && \
        rosdep install --from-paths src --ignore-src -r -y && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && rm -rf /opt/ros/px4_ws/build /opt/ros/px4_ws/log

# Add the agent binary to PATH
ENV PATH="/opt/ros/px4_ws/install/microxrcedds_agent/bin:${PATH}"

# PX4-Autopilot v1.16.0 SITL
RUN git clone https://github.com/PX4/PX4-Autopilot.git \
        --branch v1.16.0 --depth 1 --recursive \
        /opt/PX4-Autopilot \
    && cd /opt/PX4-Autopilot \
    && make px4_sitl

ENV GZ_SIM_RESOURCE_PATH=/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds

WORKDIR /root

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /opt/ros/px4_ws/install/setup.bash" >> ~/.bashrc \
    && echo "[ -f /workspaces/px4_offboard_lowlevel/install/setup.bash ] && \
        source /workspaces/px4_offboard_lowlevel/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
