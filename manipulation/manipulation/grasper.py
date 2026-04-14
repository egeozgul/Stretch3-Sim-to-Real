#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
import stretch_body.robot
import ikpy.utils
import ikpy.chain
import time
from hello_helpers.hello_misc import HelloNode
from rclpy.executors import MultiThreadedExecutor

TARGET_WRIST_ROLL  = 0.0
TARGET_WRIST_PITCH = 0.0
TARGET_WRIST_YAW   = -np.pi / 4

FALLBACK_X = +0.520
FALLBACK_Y = -0.650
FALLBACK_Z = +0.599

YAW_OFFSET = 1.06  # calibrated frame offset

GRIPPER_OPEN  =  50.0  # degrees — open
GRIPPER_CLOSE = -10.0  # degrees — closed/grasping
APPROACH_OFFSET = 0.08   # metres above object to pre-position (8 cm)
POST_GRASP_LIFT  = 0.10  # metres to lift after grasping (10 cm)


class ObjectGrasper(HelloNode):
    def __init__(self):
        super().__init__()
        super().main('object_grasper', 'object_grasper', wait_for_first_pointcloud=False)
        self.robot = stretch_body.robot.Robot()
        
        self.ik_urdf_path = '/home/willy/ament_ws/src/stretch_ros2/stretch_description/urdf/stretch_ik_modified.urdf'
        self.chain = ikpy.chain.Chain.from_urdf_file(self.ik_urdf_path)
        self.live_target_time = None
        TARGET_FRESHNESS_SEC = 1.0  # reject detections older than this
        self.live_target = None

        self.target_sub = self.create_subscription(
            PointStamped,
            '/aruco/target_base_link',
            self.target_callback,
            10
        )

        # Lock base translation
        self.chain.links[1].bounds = (-1e-6, 1e-6)
        # Fix inf bounds on fixed joints
        for i in [0, 2, 4, 10, 13, 14]:
            self.chain.links[i].bounds = (-1e-6, 1e-6)

        self.grasp_object_service = self.create_service(Trigger, '/grasp_object', self.grasp_object)

        self.get_logger().info("="*60)
        self.get_logger().info("ObjectGrasper ready — waiting for ArUco detections")
        self.get_logger().info("Listening on: /aruco/target_base_link")
        self.get_logger().info("Call service: ros2 service call /grasp_object std_srvs/srv/Trigger {}")
        self.get_logger().info("="*60)

    def target_callback(self, msg: PointStamped):
        self.live_target = (msg.point.x, msg.point.y, msg.point.z)
        self.live_target_time = time.time()  # ADD THIS
        self.get_logger().info(
            f"[ArUco] target updated → X={msg.point.x:+.3f}  Y={msg.point.y:+.3f}  Z={msg.point.z:+.3f}",
            throttle_duration_sec=1.0
        )
    def is_target_fresh(self):
        if self.live_target is None or self.live_target_time is None:
            return False
        age = time.time() - self.live_target_time
        if age > 1.0:
            self.get_logger().warn(f"Target is stale ({age:.1f}s old) — aborting")
            return False
        return True
    def get_current_configuration(self):
        self.robot.pull_status()
        def bound(name, value):
            names = [l.name for l in self.chain.links]
            idx = names.index(name)
            lo, hi = self.chain.links[idx].bounds
            return float(np.clip(value, lo, hi))

        q_lift  = bound('joint_lift',        self.joint_state.position[self.joint_state.name.index('joint_lift')])
        q_arm0  = bound('joint_arm_l0',      self.joint_state.position[self.joint_state.name.index('joint_arm_l0')])
        q_arm1  = bound('joint_arm_l1',      self.joint_state.position[self.joint_state.name.index('joint_arm_l1')])
        q_arm2  = bound('joint_arm_l2',      self.joint_state.position[self.joint_state.name.index('joint_arm_l2')])
        q_arm3  = bound('joint_arm_l3',      self.joint_state.position[self.joint_state.name.index('joint_arm_l3')])
        q_yaw   = bound('joint_wrist_yaw',   self.joint_state.position[self.joint_state.name.index('joint_wrist_yaw')])
        q_pitch = bound('joint_wrist_pitch', self.joint_state.position[self.joint_state.name.index('joint_wrist_pitch')])
        q_roll  = bound('joint_wrist_roll',  self.joint_state.position[self.joint_state.name.index('joint_wrist_roll')])

        return [0.0, 0.0, 0.0, q_lift, 0.0, q_arm3, q_arm2, q_arm1, q_arm0, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]

    def get_random_configuration(self):
        q_init = []
        for i, link in enumerate(self.chain.links):
            if i == 1:
                q_init.append(0.0)
                continue
            lo, hi = link.bounds
            if lo is None or np.isinf(lo): lo = -0.5
            if hi is None or np.isinf(hi): hi =  0.5
            if lo == hi: lo = 0.0; hi = 0.0
            q_init.append(float(np.random.uniform(lo, hi)))
        return q_init

    def open_gripper(self):
        self.get_logger().info("→ Opening gripper")
        self.move_to_pose({'joint_gripper_finger_left': GRIPPER_OPEN}, blocking=True)

    def close_gripper(self):
        self.get_logger().info("→ Closing gripper")
        self.move_to_pose({'joint_gripper_finger_left': GRIPPER_CLOSE}, blocking=True)

    def retract_arm(self):
        self.get_logger().info("→ Retracting arm")
        self.move_to_pose({'joint_arm': 0.0}, blocking=True)

    def move_to_configuration(self, q, lift_override=None):
        """Move lift + wrist only — arm stays retracted.
        lift_override lets you set a different lift height (e.g. approach height).
        """
        q_lift  = lift_override if lift_override is not None else q[3]
        q_yaw   = float(np.clip(q[9] + YAW_OFFSET, -1.75, 4.0))
        q_pitch = q[11]
        q_roll  = q[12]

        self.get_logger().info(f"→ Lift={q_lift:.3f}m  yaw={q_yaw:.3f}  pitch={q_pitch:.3f}  roll={q_roll:.3f}")
        self.move_to_pose({
            'joint_lift':        q_lift,
            'joint_wrist_yaw':   q_yaw,
            'joint_wrist_pitch': q_pitch,
            'joint_wrist_roll':  q_roll,
        }, blocking=True)

    def extend_arm(self, q):
        """Extend arm to IK solution after lift and gripper are ready."""
        q_arm = q[5] + q[6] + q[7] + q[8]
        self.get_logger().info(f"→ Extending arm to {q_arm:.3f}m")
        self.move_to_pose({'joint_arm': q_arm}, blocking=True)

    def solve_ik(self, target_point):
        """Run IK with retries. Returns q_soln or None."""
        FIXED_PITCH = -0.3
        names = [l.name for l in self.chain.links]
        pitch_idx = names.index('joint_wrist_pitch')
        self.chain.links[pitch_idx].bounds = (FIXED_PITCH - 1e-6, FIXED_PITCH + 1e-6)
        q_soln = None
        for attempt in range(150):
            self.get_logger().info(f"IK attempt {attempt+1}/150")
            try:
                q_init = self.get_current_configuration() if attempt == 0 else self.get_random_configuration()
                q_init[pitch_idx] = FIXED_PITCH
                q_soln = self.chain.inverse_kinematics(
                    target_point,
                    orientation_mode=None,
                    initial_position=q_init,
                )

                fk = self.chain.forward_kinematics(q_soln)[:3, 3]
                err = np.linalg.norm(fk - np.array(target_point))
                self.get_logger().info(f"IK error: {err:.4f}m")
                q_arm = q_soln[5] + q_soln[6] + q_soln[7] + q_soln[8]
                self.get_logger().info(f"base_trans={q_soln[1]:+.4f}m  lift={q_soln[3]:.4f}m  arm={q_arm:.4f}m")
                if err > 0.05:
                    self.get_logger().warn(f"IK error too large ({err:.4f}m) — retrying")
                    q_soln = None
                    continue

                break  # good solution found

            except ValueError as e:
                self.get_logger().warn(f"IK failed: {e} — nudging arm")
                current_arm = sum([
                    self.joint_state.position[self.joint_state.name.index(f'joint_arm_l{i}')]
                    for i in range(4)
                ])
                current_pitch = self.joint_state.position[self.joint_state.name.index('joint_wrist_pitch')]
                self.move_to_pose({'joint_arm': current_arm + 0.05}, blocking=True)
                self.move_to_pose({'joint_wrist_pitch': current_pitch - 0.05}, blocking=True)
                time.sleep(1)
                continue

        return q_soln

    def grasp_object(self, request=None, response=None):
        if not self.is_target_fresh():
            self.get_logger().error("No ArUco detection — aborting grasp")
            if response is not None:
                response.success = False
            return response

        tx, ty, tz = self.live_target
        self.get_logger().info(f"Using LIVE target: X={tx:+.3f}  Y={ty:+.3f}  Z={tz:+.3f}")

        # ── Solve IK for final grasp position ────────────────────────────────
        self.get_logger().info(f"Solving IK for grasp target: [{tx:.3f}, {ty:.3f}, {tz:.3f}]")
        q_grasp = self.solve_ik([tx, ty, tz-0.02])
        if q_grasp is None:
            self.get_logger().error("IK failed for grasp position after 150 attempts")
            if response is not None:
                response.success = False
            return response

        # ── Solve IK for approach position (same x,y but z + offset) ─────────
        approach_z = tz + APPROACH_OFFSET
        self.get_logger().info(f"Solving IK for approach target: [{tx:.3f}, {ty:.3f}, {approach_z:.3f}]")
        q_approach = self.solve_ik([tx, ty, approach_z])
        if q_approach is None:
            self.get_logger().warn("IK failed for approach position — falling back to lift offset only")
            # Fallback: reuse grasp IK, just raise the lift manually
            q_approach = None

        # ── Grasp sequence ────────────────────────────────────────────────────
        self.get_logger().info("=== GRASP SEQUENCE START ===")
        
        # Step 1: Move lift to APPROACH height + set wrist orientation
        self.get_logger().info(f"Step 1: Move to approach height ({approach_z:.3f}m) + set wrist")
        if q_approach is not None:
            self.move_to_configuration(q_approach)
        else:
            # Use grasp IK for wrist, but override lift to approach height
            approach_lift = q_grasp[3] + APPROACH_OFFSET
            self.move_to_configuration(q_grasp, lift_override=approach_lift)

        # Step 2: Open gripper
        self.get_logger().info("Step 2: Open gripper")
        self.open_gripper()
        
        # Step 3: Extend arm (to x,y position, still at approach height)
        self.get_logger().info("Step 3: Extend arm to target x,y (approach height)")
        if q_approach is not None:
            self.extend_arm(q_approach)
        else:
            self.extend_arm(q_grasp)

        # Step 4: Lower lift to grasp height (descend onto object)
        self.get_logger().info(f"Step 4: Lower lift to grasp height ({q_grasp[3]:.3f}m)")
        self.move_to_pose({'joint_lift': q_grasp[3]+0.01}, blocking=True)

        # Step 5: Close gripper
        self.get_logger().info("Step 5: Close gripper")
        self.close_gripper()

        # Step 6: Lift up with object
        post_grasp_lift = q_grasp[3] + POST_GRASP_LIFT
        self.get_logger().info(f"Step 6: Lift up to {post_grasp_lift:.3f}m")
        self.move_to_pose({'joint_lift': post_grasp_lift}, blocking=True)

        # Step 7: Retract arm
        self.get_logger().info("Step 7: Retract arm")
        self.retract_arm()

        self.get_logger().info("=== GRASP SEQUENCE COMPLETE ===")

        if response is not None:
            response.success = True
        return response


def main(args=None):
    node = ObjectGrasper()
    rclpy.spin(node, executor=MultiThreadedExecutor())


if __name__ == '__main__':
    main()