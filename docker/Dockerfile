# Use the official ROS 2 Humble + Gazebo image
# This is based on Ubuntu 22.04
FROM osrf/ros:humble-desktop-full

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Install essential tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep (init is already done in the base image)
RUN rosdep update

# Create the ROS 2 workspace directory
WORKDIR /root/ros2_ws

# Source the base ROS 2 setup in the .bashrc
# This means ROS 2 will be ready every time you open a shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Default command to start an interactive shell
CMD ["/bin/bash"]
