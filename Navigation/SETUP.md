# Stretch 3 ROS2 Navigation Setup Guide

This guide covers the full workflow for setting up SLAM mapping and autonomous navigation on a **Hello Robot Stretch 3 (SE3)** running **ROS2 Humble**, including connecting RViz from a Windows machine.

---

## Prerequisites

- Stretch 3 robot powered on and connected to the same WiFi network as your Windows machine
- SSH access to the robot
- [VcXsrv](https://sourceforge.net/projects/vcxsrv/) installed on Windows
- ROS2 Humble installed on the robot (pre-installed on Stretch 3)

---

## Step 1 — Connect to the Robot via SSH

On Windows, open PowerShell and connect:
```powershell
ssh willy@stretch-se3-<robot-id>.local
```

---

## Step 2 — Set Up RViz Display (XLaunch on Windows)

RViz needs an X server running on Windows to display the GUI over SSH.

**On Windows:**

1. Kill any existing VcXsrv instances:
```powershell
taskkill /F /IM vcxsrv.exe
```

2. Launch VcXsrv with access control disabled:
```powershell
& "C:\Program Files\VcXsrv\vcxsrv.exe" -multiwindow -ac
```
You should see a small X icon appear in the system tray.

3. Find your Windows IP address:
```powershell
ipconfig
```
Look for the **Wi-Fi adapter's IPv4 Address** (e.g. `192.168.1.6`).

**On the robot (in every new terminal):**
```bash
export DISPLAY=192.168.1.x:0.0
```

Replace `192.168.1.x` with your actual Windows IP.

---

## Step 3 — Source the Workspace

Run this in every new terminal on the robot, or add it permanently to `.bashrc`:

```bash
source ~/ament_ws/install/setup.bash
```

**To make it permanent:**
```bash
echo "source ~/ament_ws/install/setup.bash" >> ~/.bashrc
echo "export DISPLAY=192.168.1.x:0.0" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4 — SLAM Mapping

### 4.1 Free the Robot Lock
Before launching anything, make sure no other process is holding the robot lock:
```bash
stretch_free_robot_process.py
```

### 4.2 Launch SLAM
`offline_mapping.launch.py` handles the driver, lidar, SLAM toolbox and RViz all in one command. Do **not** run the driver separately.

**With keyboard teleop:**
```bash
ros2 launch stretch_nav2 offline_mapping.launch.py teleop_type:=keyboard
```

**With gamepad (Xbox controller):**
```bash
ros2 launch stretch_nav2 offline_mapping.launch.py teleop_type:=joystick
```

RViz will open on your Windows screen showing the robot and the map being built.

### 4.3 Drive the Robot to Build the Map

**Keyboard controls:**
| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `k` | Stop |
| `q/z` | Increase/decrease speed |

**Gamepad controls:**
- Hold **LB** (front left bumper) as deadman switch
- Use **right joystick** to move and rotate

**Tips for a good map:**
- Drive slowly and smoothly
- Cover all areas of the room
- Revisit previously visited spots to form loop closures
- Avoid sharp fast turns

### 4.4 Save the Map
Once the map looks complete in RViz, open a **new terminal** and run:
```bash
source ~/ament_ws/install/setup.bash
mkdir -p ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/my_map
```

This saves two files:
- `my_map.pgm` — the map image
- `my_map.yaml` — the map metadata

**Inspect the saved map:**
```bash
eog ${HELLO_FLEET_PATH}/maps/my_map.pgm
```

### 4.5 Stop Mapping
Press **Ctrl+C** in the mapping terminal to stop.

---

## Step 5 — Navigation

### 5.1 Free the Robot Lock
```bash
stretch_free_robot_process.py
```

### 5.2 Launch Navigation
```bash
source ~/ament_ws/install/setup.bash
export DISPLAY=192.168.1.x:0.0
ros2 launch stretch_nav2 navigation.launch.py \
  map:=${HELLO_FLEET_PATH}/maps/my_map.yaml \
  teleop_type:=joystick \
  mode:=navigation
```

RViz will open showing the robot on the saved map.

### 5.3 Start Navigation Nodes
In RViz, click the **"Startup"** button at the bottom left to activate all Nav2 lifecycle nodes.

### 5.4 Set Initial Robot Pose
AMCL (localization) needs to know where the robot is on the map:

1. Click **"2D Pose Estimate"** in the RViz top toolbar
2. Click on the map where the robot physically is
3. Hold and drag in the direction the robot is facing
4. Release

The robot will localize itself and the map frame will become active.

### 5.5 Enable Gamepad
After launch, fix the gamepad axis mapping:
```bash
source ~/ament_ws/install/setup.bash
ros2 param set /teleop_twist_joy_node axis_linear.x 1
ros2 param set /teleop_twist_joy_node axis_angular.yaw 0
```

Now hold **LB + right joystick** to drive manually.

### 5.6 Send Autonomous Navigation Goals
In RViz:
1. Click **"Nav2 Goal"** in the top toolbar
2. Click on the map where you want the robot to go
3. Hold and drag to set the goal orientation
4. Release — the robot will plan a path and navigate autonomously

---

## Step 6 — RViz Configuration Tips

### Show/Hide Elements
| Element | How to toggle |
|---------|--------------|
| TF gizmos (coordinate frame arrows) | Uncheck **TF** in the Displays panel |
| Robot 3D model | Check **RobotModel** in the Displays panel |
| Laser scan | Check **LaserScan** → topic: `/scan_filtered` |
| Map | Check **Map** → topic: `/map` |
| Navigation path | Check **Global Planner** |

### Fixed Frame
Make sure **Global Options → Fixed Frame** is set to `map` for navigation. If it shows an error, the map frame hasn't been initialized yet — set the 2D Pose Estimate first.

---

## Step 7 — Loading a Previously Saved Map

If you have already built and saved a map, skip the SLAM step and go straight to navigation:

```bash
stretch_free_robot_process.py
source ~/ament_ws/install/setup.bash
export DISPLAY=192.168.1.x:0.0
ros2 launch stretch_nav2 navigation.launch.py \
  map:=${HELLO_FLEET_PATH}/maps/my_map.yaml \
  teleop_type:=joystick \
  mode:=navigation
```

List all saved maps:
```bash
ls ${HELLO_FLEET_PATH}/maps/
```

---

## Quick Reference

### Full Workflow Summary
```
1. Start VcXsrv on Windows (vcxsrv.exe -multiwindow -ac)
2. SSH into robot
3. source ~/ament_ws/install/setup.bash
4. export DISPLAY=<windows-ip>:0.0
5. stretch_free_robot_process.py
6. ros2 launch stretch_nav2 offline_mapping.launch.py teleop_type:=joystick
7. Drive robot to build map
8. Save map: ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/my_map
9. Ctrl+C to stop mapping
10. stretch_free_robot_process.py
11. ros2 launch stretch_nav2 navigation.launch.py map:=... teleop_type:=joystick mode:=navigation
12. Fix gamepad axes (ros2 param set ...)
13. Set 2D Pose Estimate in RViz
14. Send Nav2 Goals
```

### Useful Commands
```bash
# Check robot driver mode
ros2 param get /stretch_driver mode

# Switch to navigation mode at runtime
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger

# Check lidar is publishing
ros2 topic hz /scan

# Check gamepad is publishing
ros2 topic echo /joy

# Check cmd_vel is receiving commands
ros2 topic echo /stretch/cmd_vel

# View TF tree
ros2 run tf2_tools view_frames

# List all ROS nodes
ros2 node list

# List all ROS topics
ros2 topic list
```

### Driver Modes
| Mode | Base control | Arm/joints | Use case |
|------|-------------|------------|----------|
| `position` | ❌ | ✅ | Arm manipulation only |
| `navigation` | ✅ velocity | ❌ | SLAM & Nav2 |
| `trajectory` | ✅ coordinated | ✅ coordinated | Whole-body control with MoveIt2 |
