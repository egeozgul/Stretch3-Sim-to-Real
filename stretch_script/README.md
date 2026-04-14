# Stretch Robot Task Scripts

## Overview

The robot workflow is split into two scripts that run in separate terminals:

- **`launch_nodes.sh`** — long-running background process that starts all ROS2 nodes
- **`run_task.sh`** — executes the task sequence (navigate to table → grasp)
- **`return_home.sh`** — returns the robot to its origin position after a task

---

## Quick Start

**Terminal 1** — start nodes and leave running:
```bash
./launch_nodes.sh
```

**Terminal 2** — once nodes are ready, run the task:
```bash
./run_task.sh
```

**Terminal 2** — to return the robot home after the task:
```bash
./return_home.sh
```

---

## launch_nodes.sh

Starts all required ROS2 nodes and prepares the robot for operation. Run this first and leave it running in its own terminal.

**What it does, in order:**
1. Frees any existing robot process
2. Reboots Dynamixel servos (clears overload errors)
3. Prompts whether to home the robot (auto-homes after 10s if no input)
4. Launches Nav2 navigation stack with the pre-built map
5. Launches the D435i depth camera
6. Launches the ArUco marker detector
7. Launches the object grasper node
8. Waits for Nav2 and the camera to be ready
9. Sets the initial pose and waits for AMCL to localize
10. Writes a ready flag (`/tmp/stretch_nodes_ready`) so other scripts know nodes are up

**Notes:**
- RViz launches automatically as part of Nav2. Requires an active SSH session with X forwarding (`ssh -X`) to display.
- All output is logged to `~/stretch_script/logs/launch_nodes_<timestamp>.log`
- Do not close this terminal while the robot is operating

---

## run_task.sh

Executes the full pick task. Run this in a second terminal after `launch_nodes.sh` is ready.

**What it does, in order:**
1. Waits for the ready flag from `launch_nodes.sh`
2. Switches to navigation mode and sets yaw tolerance
3. Resets the arm to its default position
4. Navigates the robot to the table
5. Waits for the grasp service to become available
6. Triggers the grasp

**Notes:**
- Will wait up to 120 seconds for `launch_nodes.sh` to finish startup
- Aborts with an error message if any step fails
- Logs to `~/stretch_script/logs/run_task_<timestamp>.log`

---

## return_home.sh

Returns the robot to its origin position (0, 0) after completing a task.

**What it does, in order:**
1. Waits for the ready flag from `launch_nodes.sh`
2. Switches to navigation mode and sets yaw tolerance
3. Navigates the robot back to the origin
4. Resets the arm to its default position

**Notes:**
- Can be run any time after `launch_nodes.sh` is ready, not just after `run_task.sh`
- Logs to `~/stretch_script/logs/return_home_<timestamp>.log`

---

## Requirements

- `launch_nodes.sh` must be running before `run_task.sh` or `return_home.sh`
- SSH with X forwarding (`ssh -X user@robot-ip`) required for RViz display
- Map file must exist at `${HELLO_FLEET_PATH}/maps/testing_map.yaml`
- All scripts must be executable: `chmod +x *.sh`
