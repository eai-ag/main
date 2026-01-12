FROM ubuntu:22.04

ARG TARGETARCH

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        curl \
        ca-certificates \
        gnupg \
        lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list

# Note: rpicam-apps from Raspberry Pi repo not available for Ubuntu, we'll build from libcamera only


RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    \
    libcamera-dev \
    libcamera-tools \
    \
    gstreamer1.0-tools \
    \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev \
    gstreamer1.0-nice \
    libglib2.0-dev \
    \
    libssl-dev \
    libgstreamer-plugins-bad1.0-dev \
    \
    libsoup2.4-dev \
    libjson-glib-dev \
    libnice-dev \
    libsrtp2-dev \
    libopus-dev \
    libvpx-dev \
    libx264-dev \
    \
    libpango1.0-dev \
    \
    git \
    ninja-build \
    python3-pip \
    python3-jinja2 \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Install newer meson and required Python packages via pip
RUN pip3 install meson ply

# Build and install gstreamer-libcamera plugin from source
RUN git clone https://github.com/raspberrypi/libcamera.git /tmp/libcamera && \
    cd /tmp/libcamera && \
    meson setup build --prefix=/usr -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dgstreamer=enabled && \
    ninja -C build install && \
    rm -rf /tmp/libcamera

# Install ROS2 Humble
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-gscam \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Create colcon workspace and copy ROS2 packages
RUN mkdir -p /colcon_ws/src
COPY ros2_ws/src /colcon_ws/src

# Build the workspace
WORKDIR /colcon_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Update environment to include workspace
ENV COLCON_PREFIX_PATH=/colcon_ws/install:/opt/ros/humble
ENV AMENT_PREFIX_PATH=/colcon_ws/install:/opt/ros/humble
ENV LD_LIBRARY_PATH=/colcon_ws/install/lib:/opt/ros/humble/lib
ENV PYTHONPATH=/colcon_ws/install/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages

# Source ROS2 and workspace setup in bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /colcon_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /colcon_ws