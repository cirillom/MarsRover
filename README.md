This Project was developed by Students at University of Applied Science Hof. Team consisted of International Students across India, Mexico, Brazil and Cameroon.

Our main Goal was to make a map of the surrounding, so that a UGV can traverse between two points autonomously, after finding the path of least resistance.

Hardware Requirements:
UGV: AgileX Scout Mini
LIDAR: RS Helios 16P
Computer: Jetson Orin Nano
Camera: ELB Camera(It was used as a passive sensor, only used for seeing what is in front of the rover)

Software Requirements:
OS: Ubuntu 22.04 GUI
Software: ROS2 Humble Hawkbills, Gazebo Fortress, RVIZ2

Final Results:
In Simulation we were able to complete the project, we used SLAM for making the map in real time and Nav2 for autonomously traversal.

Actual Implementation:
We were able to implement SLAM but not Nav2. You can run our physical setup if you follow "master_launch.md"
