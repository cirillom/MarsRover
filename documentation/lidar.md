LIDAR: **RS helios 16P**

Software Requirements:  -> Ubuntu 22.04
                        -> ROS2 Humble

Harware Requirements: -> Ethernet Connectivity between the LIDAR and Host Device

Miscellaneous:
LIDAR IP: 192.168.26.60

Setup:
Make a workspace for the LIDAR "rslidar_ws". Follow the official guide: https://github.com/Kyronxu/RS_HELIOS/blob/main/README.md. 

In the Terminal:

1. Change the IP address of the Host Device to 192.168.26.20, LIDAR sends the data on this IP:
```
sudo ip addr add 192.168.26.20 dev enP8p1s0
sudo ip link set up dev enP8p1s0
```

2. Check if the IP address is up:
```
ip a                           # Should show IP of enP8p1s0 as set above
ip neigh                       # Should show enP8p1s0 as STALE or REACHABLE 
```

3. ***OPTIONAL*** -> To check if the LIDAR is being detected and sending the UDP packets respectively:                          
```               
sudo apt install wireshark     # Check if enP8p1s0 is STALE or REACHABLE
sudo tshark -i enP8p1s0 -f "udp" 
```

4. Activate ROS and Launch:
```
source /opt/ros/humble/bash
cd rs_lidar
source install/setup.bash
ros2 launch rslidar_sdk start.py 
```

5. Visualize the Point Cloud:
```
ros2 topic list               #You should see /rslidar_points
rviz2
```                       
In RIVZ2 -> In Global Options, set Fixed Frames to **rslidar** and Under PointCloud2 set Topic to **/rslidar_points**

You should see the Points on the right Visualization Screen

