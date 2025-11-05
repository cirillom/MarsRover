# Simulation

In order to test and validate algorithms in a controlled environment, we provide a simulation framework that mimics real-world scenarios. This section outlines the setup and usage of the simulation environment.

## Prerequisites
Before setting up the communication, ensure you have the following prerequisites:
- A computer with Ubuntu 22.04 installed (WSL 2.0 also works).
- ROS 2 Humble Hawksbill installed on your system. Follow the [ROS 2 Installation Guide](ros2.md) if you haven't done this yet.

## Gazebo Installation
To run the simulation, you need to install Gazebo, a popular robotics simulator.

```bash
    sudo apt install ros-humble-ros-gz
```

Test with:
```bash
    ign gazebo shapes.sdf
```

### WSL 2.0 Users
If you're using WSL, the program will probably crash, to fix this you need to run these commands:
```bash
    add-apt-repository ppa:kisak/turtle && apt update && apt upgrade -y
```

And then always run this command before launching Gazebo:
```bash
    export GALLIUM_DRIVER=d3d12
```

## Installing the Scout Mini Simulation Package
You first need the navigation2 package:
```bash
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

Then you can create a workspace and clone the Scout Mini simulation package:
```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/mattiadutto/ugv_gazebo_sim
```

Build the workspace and source it:
```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
```

## Launching the Simulation
To launch the Scout Mini simulation in Gazebo, use the following command:
```bash
    ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py
```

If you want to test movement you can use the teleop twist keyboard:
```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/scout/cmd_vel
```
This will allow you to control the Scout Mini robot in the Gazebo simulation environment using your keyboard.
