# Stretch 3 ROS2 Navigation & SLAM Setup — Troubleshooting Guide

This document covers common errors encountered when setting up SLAM and Nav2 navigation on a **Hello Robot Stretch 3 (SE3)** running **ROS2 Humble**, and how to diagnose and fix them.

---

## Table of Contents
1. [Workspace Not Sourced](#1-workspace-not-sourced)
2. [Robot Process Lock](#2-robot-process-lock)
3. [RViz Display Not Found (X11 Forwarding)](#3-rviz-display-not-found-x11-forwarding)
4. [Frame \[map\] Does Not Exist](#4-frame-map-does-not-exist)
5. [ROS Daemon Crashed](#5-ros-daemon-crashed)
6. [Gamepad Not Moving the Robot](#6-gamepad-not-moving-the-robot)
7. [RobotModel Mesh Loading Errors in RViz](#7-robotmodel-mesh-loading-errors-in-rviz)

---

## 1. Workspace Not Sourced

### Symptom
```
Package 'stretch_core' not found: "package 'stretch_core' not found, searching: ['/opt/ros/humble']"
```
or
```
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
```

### Cause
The Stretch ROS2 workspace hasn't been sourced in the current terminal.

### Fix
```bash
source ~/ament_ws/install/setup.bash
```

### Permanent Fix
Add it to `.bashrc` so every terminal sources it automatically:
```bash
echo "source ~/ament_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### How to Check
```bash
ros2 pkg list | grep stretch
```
If stretch packages appear, the workspace is sourced correctly.

---

## 2. Robot Process Lock

### Symptom
```
Another process is already using Stretch. Try running "stretch_free_robot_process.py"
[FATAL] stretch_driver: Robot startup failed.
```

### Cause
Another process is holding the Stretch hardware lock. This often happens when:
- A previous session wasn't cleanly terminated
- The driver was launched twice
- `offline_mapping.launch.py` was launched while a separate `stretch_driver` was already running (these conflict because `offline_mapping.launch.py` includes its own driver)

### Fix
```bash
stretch_free_robot_process.py
```

If that doesn't work, force kill all ROS processes:
```bash
pkill -9 -f stretch_driver
pkill -9 -f ros2
pkill -9 -f python3
stretch_free_robot_process.py
```

### How to Check
```bash
ps aux | grep -E "stretch|ros2" | grep -v grep
```
Should return nothing before launching.

### Important Note
`offline_mapping.launch.py` includes its own `stretch_driver` — **do not** run a separate driver before launching it. It must be the only thing running.

---

## 3. RViz Display Not Found (X11 Forwarding)

### Symptom
```
qt.qpa.xcb: could not connect to display
Authorization required, but no authorization protocol specified
```

### Cause
RViz needs a display to render on. When SSH-ing from Windows, X11 forwarding must be configured correctly.

### Fix

**On Windows:**
1. Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Kill any existing instances: `taskkill /F /IM vcxsrv.exe` in PowerShell
3. Launch with access control disabled:
```powershell
& "C:\Program Files\VcXsrv\vcxsrv.exe" -multiwindow -ac
```

**On the robot:**
```bash
export DISPLAY=<your-windows-ip>:0.0
```

Find your Windows IP with `ipconfig` — look for the Wi-Fi adapter's IPv4 address.

### How to Check
```bash
echo $DISPLAY
xhost +
```
If `xhost` fails with "unable to open display", VcXsrv is not running or blocking connections.

### Tip
Add the export to your launch command every session, or add it to `.bashrc`:
```bash
echo "export DISPLAY=192.168.x.x:0.0" >> ~/.bashrc
```

---

## 4. Frame \[map\] Does Not Exist

### Symptom
In RViz:
```
Frame [map] does not exist
```
In terminal:
```
Invalid frame ID "map" passed to canTransform
slam_toolbox: Message Filter dropping message: frame 'laser'
```

### Cause
SLAM toolbox hasn't initialized the `map` frame yet. This is almost always caused by the `stretch_driver` failing to start due to the **robot process lock** (see Issue #2).

### Diagnosis
```bash
# Check if odom->base_link transform exists
ros2 run tf2_ros tf2_echo map odom

# View the full TF tree
ros2 run tf2_tools view_frames

# Check if map topic is publishing
ros2 topic hz /map
```

If `odom` frame doesn't exist in the TF tree, the driver failed to start.

### Fix
1. Kill everything and free the robot lock (see Issue #2)
2. Launch `offline_mapping.launch.py` as the **only** process — it handles the driver internally:
```bash
stretch_free_robot_process.py
source ~/ament_ws/install/setup.bash
export DISPLAY=<windows-ip>:0.0
ros2 launch stretch_nav2 offline_mapping.launch.py teleop_type:=keyboard
```

---

## 5. ROS Daemon Crashed

### Symptom
```
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
```
Even after sourcing the workspace, `ros2` commands fail.

### Fix
```bash
ros2 daemon stop
ros2 daemon start
```

---

## 6. Gamepad Not Moving the Robot

### Symptom
Gamepad buttons publish on `/joy` but the robot doesn't move. `/stretch/cmd_vel` shows no messages.

### Diagnosis
```bash
# Check if joy messages are publishing
ros2 topic echo /joy

# Check if cmd_vel is receiving anything
ros2 topic echo /stretch/cmd_vel

# Check teleop node info
ros2 node info /teleop_twist_joy_node

# Check current axis/button mapping
ros2 param dump /teleop_twist_joy_node
```

### Cause
The `teleop_twist_joy_node` has incorrect axis mappings. The default config sets `axis_linear.x: 4` which conflicts with the enable button (`enable_button: 4`). The correct axes for the Xbox controller on Stretch are:
- **Axis 0** — left/right (angular yaw)
- **Axis 1** — forward/backward (linear x)
- **Button 4** — LB deadman switch (enable button)

### Fix
```bash
ros2 param set /teleop_twist_joy_node axis_linear.x 1
ros2 param set /teleop_twist_joy_node axis_angular.yaw 0
```

### How to Find Button/Axis Mapping
Hold a button and run:
```bash
ros2 topic echo /joy --once
```
The button that shows `1` in the `buttons` array is the index of that button.

### Permanent Fix
Create a custom joystick config file at `~/ament_ws/src/stretch_ros2/stretch_nav2/config/joystick.yaml`:
```yaml
teleop_twist_joy_node:
  ros__parameters:
    enable_button: 4
    axis_linear:
      x: 1
    axis_angular:
      yaw: 0
```

---

## 7. RobotModel Mesh Loading Errors in RViz

### Symptom
In RViz, RobotModel shows:
```
Status: Error
URDF: Errors loading geometries
```

### Diagnosis
```bash
# Check what mesh paths the URDF uses
ros2 param get /robot_state_publisher robot_description | grep -o 'filename="[^"]*"' | head -5

# Check for missing mesh files
ros2 param get /robot_state_publisher robot_description | grep -o 'filename="[^"]*"' | sed 's/filename="//;s/"//' | while read f; do [ -f "$f" ] || echo "MISSING: $f"; done

# Check if meshes directory is populated
ls ~/ament_ws/install/stretch_description/share/stretch_description/meshes/

# Check for detailed errors
ros2 launch stretch_nav2 navigation.launch.py ... 2>&1 | grep -i "mesh\|geometry\|stl\|dae"
```

### Root Causes & Fixes

#### Cause 1: Meshes directory is empty
The `stretch_description` package doesn't install meshes during build for newer robot batches (e.g. `sza`).

**Fix:** Copy meshes from the robot's exported URDF:
```bash
cp /home/willy/stretch_user/<robot-name>/exported_urdf/meshes/* \
  ~/ament_ws/install/stretch_description/share/stretch_description/meshes/
```

Find your robot name with:
```bash
stretch_params.py | grep batch
```

#### Cause 2: URDF uses relative paths (`./meshes/`)
RViz can't resolve relative mesh paths.

**Fix:** Update URDF to use `package://` URIs:
```bash
# Fix the calibrated URDF
sed -i 's|filename="./meshes/|filename="package://stretch_description/meshes/|g' \
  ~/ament_ws/install/stretch_description/share/stretch_description/urdf/stretch.urdf

# Fix the uncalibrated URDF (fallback)
sed -i 's|filename="./meshes/|filename="package://stretch_description/meshes/|g' \
  /home/willy/.local/lib/python3.10/site-packages/stretch_urdf/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf
```

#### Cause 3: Wrong URDF being loaded
The launch may use the uncalibrated URDF from `stretch_urdf` instead of the calibrated one.

**Check which URDF is loaded at runtime:**
```bash
ros2 param get /robot_state_publisher robot_description | grep -o 'filename="[^"]*"' | head -3
```
If paths are still relative after restarting, multiple `robot_state_publisher` instances may be running:
```bash
ps aux | grep robot_state_publisher | grep -v grep
```
Kill all and relaunch cleanly.

---

## General Launch Order

### SLAM Mapping
```bash
# Free robot lock first
stretch_free_robot_process.py

# Launch mapping (handles driver internally)
source ~/ament_ws/install/setup.bash
export DISPLAY=<windows-ip>:0.0
ros2 launch stretch_nav2 offline_mapping.launch.py teleop_type:=keyboard
```

Drive the robot around, then save the map:
```bash
mkdir -p ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/my_map
```

### Navigation
```bash
# Stop mapping first (Ctrl+C), then:
stretch_free_robot_process.py
source ~/ament_ws/install/setup.bash
export DISPLAY=<windows-ip>:0.0
ros2 launch stretch_nav2 navigation.launch.py \
  map:=${HELLO_FLEET_PATH}/maps/my_map.yaml \
  teleop_type:=joystick \
  mode:=navigation

# Fix gamepad axes after launch
ros2 param set /teleop_twist_joy_node axis_linear.x 1
ros2 param set /teleop_twist_joy_node axis_angular.yaw 0
```

In RViz:
1. Press **Startup** button (bottom left)
2. Use **2D Pose Estimate** to set robot's initial position on the map
3. Use **Nav2 Goal** to send autonomous navigation goals

---

## Robot Info Commands

```bash
# Check current driver mode
ros2 param get /stretch_driver mode

# Switch modes at runtime
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger

# Check active topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames

# Check lidar
ros2 topic echo /scan --once

# Check robot batch
stretch_params.py | grep batch
```
