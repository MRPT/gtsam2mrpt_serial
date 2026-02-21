# Start from Ubuntu 24.04 with ROS 2 Rolling base
FROM ros:rolling-ros-base

# Install dev tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    ninja-build \
    gdb \
    clangd \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Setup rosdep (root + ubuntu user)
RUN sudo rosdep init || true \
    && rosdep update || true \
    && sudo -u ubuntu rosdep update || true

# Add ROS2 setup to bashrc for the default ubuntu user
RUN echo "source /opt/ros/rolling/setup.bash" >> /home/ubuntu/.bashrc

# Optional: helper script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/rolling/setup.bash\n\
sudo apt-get update\n\
rosdep install --from-paths . --ignore-src -r -y' \
    > /opt/ros/setup-dev.sh && chmod +x /opt/ros/setup-dev.sh

# Switch to the ubuntu user (already exists in the base image)
USER ubuntu
