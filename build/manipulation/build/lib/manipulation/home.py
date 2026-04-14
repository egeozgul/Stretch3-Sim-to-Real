#!/usr/bin/env python3
import math
from hello_helpers.hello_misc import HelloNode
import rclpy

ROTATE_DEG = 0  # negative = clockwise (right turn)

class ResetPose(HelloNode):
    def __init__(self):
        super().__init__()
        super().main('reset_pose', 'reset_pose', wait_for_first_pointcloud=False)

        self.get_logger().info("Moving to default pose...")
        self.move_to_pose({
            'joint_lift':                0.5,
            'joint_arm':                 0.0,
            'joint_wrist_yaw':           0.0,
            'joint_wrist_pitch':         0.0,
            'joint_wrist_roll':          0.0,
            'joint_gripper_finger_left': 0.0,
        }, blocking=True)
        self.get_logger().info("Arm reset done.")

        self.get_logger().info(f"Rotating base {ROTATE_DEG}°...")
        self.move_to_pose({
            'rotate_mobile_base': math.radians(ROTATE_DEG),
        }, blocking=True)
        self.get_logger().info("Done.")


def main(args=None):
    node = ResetPose()


if __name__ == '__main__':
    main()