# NVIDIA Jetson Xavier NX

The Jetson is basically a small computer that can run Ubuntu and ROS 2. It can be used as the main controller for the Scout Mini robot, providing more processing power for advanced applications such as computer vision and machine learning.

Your first step should be getting a clean installation, by default this Xavier NX comes with Ubuntu 20.04, but we need Ubuntu 22.04 to be able to install ROS 2 Humble Hawksbill. The problem is that NVIDIA does not provide an official image for Ubuntu 22.04 yet, so we will use a unofficial guide in order to upgrade to Ubuntu 22.04.

[Clean installation guide for Jetson Xavier NX](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#prepare)
[Jetson Xavier NX sd card image](https://developer.nvidia.com/assets/embedded/secure/tools/files/jetpack-sdks/jetpack-5.0.1-dp/jp501dp-xnx-sd-card-image-b118/jetson-nx-jp501dp-sd-card-image.zip)

[Ubuntu 22.04 upgrade guide for Jetson Xavier NX](https://forums.developer.nvidia.com/t/guide-upgrading-jetson-xavier-nx-to-ubuntu-22-04-for-ros2-humble-support-with-known-issues/332820)

