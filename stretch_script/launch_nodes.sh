#!/bin/bash

sudo ./kill.sh

source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash

LOG_FILE=~/stretch_script/logs/launch_nodes_$(date +%Y%m%d_%H%M%S).log
mkdir -p ~/stretch_script/logs
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== launch_nodes.sh started at $(date) ==="

export DISPLAY=localhost:10.0
export LIBGL_ALWAYS_SOFTWARE=1

READY_FLAG=/tmp/stretch_nodes_ready
rm -f "$READY_FLAG"

cleanup() {
  echo "=== Cleaning up... ==="
  rm -f "$READY_FLAG"
  kill $NAV_PID $CAMERA_PID $ARUCO_PID $GRASPER_PID 2>/dev/null
  pkill -f "navigation.launch.py"
  pkill -f "d435i_high_resolution"
  pkill -f "tf.py"
  pkill -f "altobject"
  stretch_free_robot_process.py || sudo -E env PATH=$PATH stretch_free_robot_process.py
  echo "=== Cleanup done ==="
  exit 0
}
trap cleanup SIGINT SIGTERM

# ── Free robot process ─────────────────────────────────────────────────────
echo "Freeing robot process..."
stretch_free_robot_process.py

# ── Home robot if needed ───────────────────────────────────────────────────
echo "Checking if robot needs homing..."
if ! stretch_robot_home_check.py > /dev/null 2>&1; then
  echo "Robot not homed. Homing now (this may take ~30s)..."
  stretch_robot_home.py
  echo "Homing complete."
else
  echo "Robot already homed, skipping."
fi

# ── Launch all nodes ───────────────────────────────────────────────────────
echo "Launching Nav2..."
ros2 launch stretch_nav2 navigation.launch.py \
  map:=${HELLO_FLEET_PATH}/maps/testing_map.yaml \
  teleop_type:=joystick \
  mode:=navigation &
NAV_PID=$!

echo "Launching D435i camera..."
ros2 launch stretch_core d435i_high_resolution.launch.py >> "$LOG_FILE" 2>&1 &
CAMERA_PID=$!

echo "Launching ArUco detector..."
python3 ~/tf.py >> "$LOG_FILE" 2>&1 &
ARUCO_PID=$!

echo "Launching Grasper..."
ros2 run stretch_perception altobject >> "$LOG_FILE" 2>&1 &
GRASPER_PID=$!


# ── Wait for Nav2 ──────────────────────────────────────────────────────────
echo "Waiting for Nav2 action server..."
until ros2 action info /navigate_to_pose > /dev/null 2>&1; do sleep 1; done
echo "Nav2 is up!"

# ── Wait for D435i ─────────────────────────────────────────────────────────
echo "Waiting for D435i camera..."
CAM_TIMEOUT=30; CAM_COUNT=0
until ros2 topic echo /camera/color/image_raw --once > /dev/null 2>&1; do
  sleep 1; CAM_COUNT=$((CAM_COUNT + 1))
  if [ $CAM_COUNT -ge $CAM_TIMEOUT ]; then
    echo "ERROR: Camera failed to start after ${CAM_TIMEOUT}s. Aborting."
    cleanup
  fi
done
echo "Camera ready!"

# ── Set initial pose and wait for AMCL to localize ────────────────────────
echo "Setting initial pose..."
~/stretch_script/set_initial_pose.sh

echo "Waiting for AMCL to localize..."
AMCL_TIMEOUT=30; AMCL_COUNT=0
until ros2 topic echo /amcl_pose --once > /dev/null 2>&1; do
  sleep 1; AMCL_COUNT=$((AMCL_COUNT + 1))
  if [ $AMCL_COUNT -ge $AMCL_TIMEOUT ]; then
    echo "ERROR: AMCL failed to localize after ${AMCL_TIMEOUT}s. Aborting."
    cleanup
  fi
done
echo "AMCL localized!"

# ── Signal readiness ───────────────────────────────────────────────────────
echo "=== All nodes ready. Writing flag: $READY_FLAG ==="
touch "$READY_FLAG"

# ── Stay alive ─────────────────────────────────────────────────────────────
wait
