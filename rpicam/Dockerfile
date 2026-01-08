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