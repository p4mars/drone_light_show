ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop AS ros_user_setup

# Create a non-root user
ARG ROS_USER=ros
ARG ROS_USER_UID=1000
ARG ROS_USER_GID=$ROS_USER_UID

# Create and switch to user
RUN groupadd --gid $ROS_USER_GID $ROS_USER \
    && useradd -s /bin/bash --uid $ROS_USER_UID --gid $ROS_USER_GID -m $ROS_USER \
    # Add sudo support for non-root user.
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo $ROS_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$ROS_USER \
    && chmod 0440 /etc/sudoers.d/$ROS_USER

USER $ROS_USER

# Create the workspace directory
RUN mkdir -p /home/ros/ros2_ws/src

###########################################

FROM ros_user_setup AS dev

# Install some dependencies packages
RUN sudo apt-get update && sudo apt-get upgrade -y \
    && sudo apt-get install -y --no-install-recommends \
    # install build tools
    alsa-utils \
    apt-rdepends \
    build-essential \
    git \
    cmake \
    # install useful tools
    ccache \
    wget \
    curl \
    htop \
    vim \
    clang-format \
    # install ros specifics
    libpoco-dev \
    libprotobuf-dev \
    protobuf-compiler \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \

    # apt clean-up
    && sudo apt-get autoremove -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install tools for linting
RUN sudo pip3 install --no-cache \
    black==23.9.1 \
    isort==5.12.0 \
    flake8==6.1.0 \
    pep257==0.6.0 \
    colorama \
    colcon-lint
    # colorama is required by flake8 for colorization

# solve issue where the numpy version is binary incompatible
RUN sudo pip3 install --no-cache --upgrade numpy

RUN rosdep update

# Install Micro-XRCE-DDS-Agent
WORKDIR /home/ros
RUN git clone -b v2.4.2 https://github.com/eProsima/Fast-DDS.git \
    && cd Fast-DDS \
    && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && sudo make install \
    && sudo ldconfig /usr/local/lib/

###########################################

FROM dev AS simulation

# Install Gazebo, ROS packages, and PX4-Gazebo-ROS2 Bridge
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
    gz-fortress \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-ign-bridge \
    ros-${ROS_DISTRO}-ros-ign-gazebo \ 
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-px4-msgs \
    # PX4-Gazebo-ROS2 Bridge Added
    # apt clean-up
    && sudo apt-get autoremove -y \
    && sudo rm -rf /var/lib/apt/lists/*

###########################################

# Install PX4 Autopilot
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ninja-build \
    exiftool \
    python3-jinja2 \
    python3-setuptools \
    python3-pandas \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-opencv \
    python3-empy \
    python3-toml \
    python3-click \
    && pip3 install pyulog pyquaternion pyserial \
    && sudo apt clean

# Clone PX4 repository
WORKDIR /home/ros
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git

# Install PX4 dependencies
WORKDIR /home/ros/PX4-Autopilot
RUN bash ./Tools/setup/ubuntu.sh

# Build PX4 SITL
RUN make px4_sitl

# Setup entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bin/bash"]