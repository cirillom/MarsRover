# Scout Mini Simulation Development Environment

This project uses Docker to create a consistent ROS2 Humble environment for simulating the Scout Mini with simple controls.

# 1. Prerequisites
1. Install https://docs.docker.com/engine/install/.

2. Install https://docs.docker.com/compose/install/ (comes with Docker Desktop).

3. (Linux Only) Add user to docker group: 
`sudo usermod -aG docker $USER`.
Log out and log back in for this to take effect.

4. (Windows Only) Install an X-Server:
- We recommend [VcXsr](VcXsrvhttps://vcxsrv.com/).
- During setup ("XLaunch"), accept all defaults except for the "Extra settings" screen.
- On "Extra settings," check "Disable access control". This is essential.
- Finish the setup and let it run in your system tray.0

# 2. First-Time Setup (All Users)
1. Clone this repository.

2. `cd` into the repository folder.

3. Clone the simulation code into this same folder:
```bash
git clone https://github.com/mattiadutto/ugv_gazebo_sim.git
```

4. (Linux Only) Allow GUI connections from Docker:
```bash
xhost +local:docker
```

5. Build the Docker environment (takes a few minutes):

```bash
# On Linux only!
docker-compose build
# On Windows only!
docker-compose -f docker-compose.windows.yml build
```
# 3. How to Run the Simulation
## Terminal 1: Launch Gazebo
1. Start the container in the background:

```bash
# On Linux only!
docker-compose up -d
# On Windows only!
docker-compose -f docker-compose.windows.yml up -d
```

2. Enter the container's shell:
```bash
docker exec -it scout_sim_container /bin/bash
``` 

Inside the container, run the first-time build:
```bash
# (First time only) Install package-specific dependencies
rosdep install --from-paths src --ignore-src -r -y
# (First time only) Build the workspace
colcon build
```
Every time, source and launch the simulation:


```bash
. install/setup.bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py
```

## Terminal 2: Run Teleop Controller
1. Open a new terminal window.
2. Enter the same running container:

```bash
docker exec -it scout_sim_container /bin/bash
```

Inside the container, source your workspace:

```bash
. install/setup.bash
```

Run the keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/scout_mini/cmd_vel
```

5. Click on this terminal and use the keys (i, u, o, etc.) to drive the robot.

## 4. How to Stop
1. Press Ctrl+C in your terminals.
2. To stop the container, run from the host.:
```bash
# On Linux only
docker-compose down
# On Windows only
docker-compose -f docker-compose.windows.yml down
```