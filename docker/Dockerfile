# Base Image
FROM ubuntu:22.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV ACCEPT_EULA=Y

# Install essential build tools and prerequisites
# git-lfs, build-essential, gcc-11, g++-11, cmake are required for Isaac Sim source build
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    git \
    git-lfs \
    build-essential \
    gcc-11 \
    g++-11 \
    cmake \
    python3 \
    python3-pip \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Set GCC/G++ 11 as default (priority 200)
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200

# Initialize Git LFS
RUN git lfs install

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# NVIDIA Environment Variables for Container Toolkit
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display,video

# Install missing UI/Vulkan dependencies
RUN apt-get update && apt-get install -y \
    zenity \
    vulkan-tools \
    libvulkan1 \
    libgl1 \
    libglib2.0-0 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble and Cyclone DDS
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ackermann-msgs \
    ros-humble-autoware-control-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set RMW implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Allow running as root
ENV OMNI_KIT_ALLOW_ROOT=1

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Clone Isaac Sim repository
WORKDIR /root
RUN git clone -b develop https://github.com/isaac-sim/IsaacSim.git isaacsim && \
    cd isaacsim && \
    git lfs install && \
    git lfs pull

# Set working directory to repo
WORKDIR /root/isaacsim

# Build Isaac Sim inside the image
# We pipe "yes" to accept the EULA automatically
# Note: Ensure this step effectively builds without GPU requirement
RUN echo "yes" | ./build.sh

# Copy project files
COPY . /workspace/autoware_off-road_sim

# Setup Entrypoint
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
