#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash
LOG_FILE=~/stretch_script/logs/place_object_$(date +%Y%m%d_%H%M%S).log
mkdir -p ~/stretch_script/logs
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== place_object.sh started at $(date) ==="

# ── Switch to position mode ────────────────────────────────────────────────
echo "Switching to position mode..."
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger {}
sleep 1

# ── Extend arm to table position ──────────────────────────────────────────
echo "Extending arm to place position..."
python3 - <<'EOF'
import rclpy
from hello_helpers.hello_misc import HelloNode

class PlaceObject(HelloNode):
    def __init__(self):
        super().__init__()
        super().main('place_object', 'place_object', wait_for_first_pointcloud=False)

        self.get_logger().info("Centering wrist yaw...")
        self.move_to_pose({'joint_wrist_yaw': 0.95}, blocking=True)

        self.get_logger().info("Moving to place position...")
        self.move_to_pose({
            'joint_lift': 0.9118,
            'joint_arm':  0.25,
        }, blocking=True)
        self.get_logger().info("Place position reached.")

        self.get_logger().info("Opening gripper to release object...")
        self.move_to_pose({'joint_gripper_finger_left': 0.6}, blocking=True)
        self.get_logger().info("Gripper opened, object released.")

        self.get_logger().info("Retracting arm...")
        self.move_to_pose({'joint_arm': 0.0}, blocking=True)
        self.get_logger().info("Arm retracted.")

        self.get_logger().info("Done.")

def main():
    node = PlaceObject()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ] && [ $EXIT_CODE -ne 139 ]; then
  echo "ERROR: Place sequence failed (exit $EXIT_CODE). Aborting."
  exit 1
fi
echo "Object placed successfully."

# ── Reset arm to travel position ──────────────────────────────────────────
echo "Resetting arm to travel position..."
python3 ~/test1.py; EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ] && [ $EXIT_CODE -ne 139 ]; then
  echo "ERROR: Arm reset failed (exit $EXIT_CODE)."
  exit 1
fi
echo "Arm reset complete."
echo "=== place_object.sh finished at $(date) ==="
