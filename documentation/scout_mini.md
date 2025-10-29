# Scout Mini Communication

The Scout Mini robot uses ROS 2 Humble Hawksbill and USB2CAN for communication.

## Prerequisites
Before setting up the communication, ensure you have the following prerequisites:
- A computer with Ubuntu 22.04 installed.
- ROS 2 Humble Hawksbill installed on your computer. Follow the [ROS 2 Installation Guide](ros2.md) if you haven't done this yet.
- USB2CAN adapter connected to your computer.
- Scout Mini robot powered on.

## Setting Up Communication
1. Install the following dependencies:
    ```bash
    sudo apt install build-essential git cmake libasio-dev ros-humble-teleop-twist-keyboard
    ```

2. Clone the Scout Mini ROS 2 package:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/westonrobot/ugv_sdk.git
    git clone https://github.com/westonrobot/scout_ros2.git
    cd ..
    colcon build
    source install/setup.bash
    ```
3. Connect to CAN bus using the USB2CAN adapter:
    ```bash
    sudo modprobe gs_usb
    sudo ip link set can0 up type can bitrate 500000
    ```
4. Launch the Scout Mini ROS 2 node:
    ```bash
    ros2 launch scout_base scout_base.launch.py
    ```
5. You can now control the Scout Mini using the teleop twist keyboard:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
