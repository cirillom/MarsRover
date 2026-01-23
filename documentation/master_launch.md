Overview
This guide provides step-by-step instructions to launch the Scout robot with LIDAR and SLAM (Simultaneous Localization and Mapping) system. The setup requires three separate terminal windows to run different components simultaneously.


System Requirements:

Robot: Agilex Scout mini
Sensor: RS Helios 16P LIDAR sensor
Computer: Jetson Orin Nano w/ Ubuntu 22.04 GUI
Software: ROS2 Humble

# Setting up ROS2

source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build
source install/setup.bash


# Activates the Rover Communication and launches LIDAR and SLAM

sudo modprobe gs_usb
sudo ip link set can1 up type can bitrate 500000
ros2 launch scout_startall master_launch.py


#Launching Camera
In other terminal:

cd ros2_ws
rover camera 

#You will see an IP address with a port number, paste it into a Browser to see camera feed

#Visualizing Data
In other terminal:
rviz2

#Add suitable "topics" like PointCloud2, map, scout_description in the RVIZ to visualize map, rover and point cloud
