FROM debian:trixie

ARG TARGETARCH

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        curl \
        ca-certificates \
        gpg \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "$TARGETARCH" = "arm64" ]; then \
        curl -fsSL http://archive.raspberrypi.org/debian/raspberrypi.gpg.key | gpg --dearmor -o /etc/apt/trusted.gpg.d/raspi.gpg && \
        echo "deb http://archive.raspberrypi.org/debian/ $(sh -c '. /etc/os-release; echo $VERSION_CODENAME') main" \
            > /etc/apt/sources.list.d/raspi.list && \
        echo -e "[global]\nextra-index-url=https://www.piwheels.org/simple" > /etc/pip.conf; \
        apt-get update && \
        DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        rpicam-apps && \
        rm -rf /var/lib/apt/lists/*; \
    fi


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
    gstreamer1.0-libcamera \
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
    && rm -rf /var/lib/apt/lists/*


# Install ROS2 Humble
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    python3-rosdep2 \
    python3-colcon-core \
    vcstool \
    build-essential \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/ros2_humble/src && cd ~/ros2_humble && \
    wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos && \
    vcs import src < ros2.repos && \
    rosdep update && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive rosdep install  --default-yes  --from-paths src --ignore-src --rosdistro humble -y -r --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool ignition-math6 ignition-cmake2 ignition-common3 ignition-transport8" || true && \
    curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | bash && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    git clone -b ros2 https://github.com/ros-drivers/gscam src/gscam
    
RUN cd /root/ros2_humble && \
    python3 -m colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip-regex '.*rviz.*|.*qt.*|.*gz.*|.*ignition.*|.*turtlesim.*|.*ogre.*'

RUN cd /root/ros2_humble && rm -rf build log src
# # Install gscam
# RUN cd ~/ros2_humble && \
#     git clone -b ros2 https://github.com/ros-drivers/gscam src/gscam && \
#     python3 -m colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-select gscam