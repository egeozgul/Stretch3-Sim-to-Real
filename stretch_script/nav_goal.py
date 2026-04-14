import argparse
import math
import time
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger

YAW_TOLERANCE_DEG = 3.0
XY_TOLERANCE_M    = 0.05


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self._client       = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client  = ActionClient(self, Spin, 'spin')
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._nav_mode_client  = self.create_client(Trigger, '/switch_to_navigation_mode')
        self._current_x = self._current_y = self._current_yaw = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

    # ------------------------------------------------------------------ helpers

    def _amcl_cb(self, msg):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        z, w = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self._current_yaw = math.degrees(2 * math.atan2(z, w))

    def _angle_diff(self, a, b):
        return (a - b + 180) % 360 - 180

    def _spin_executor(self, timeout=5.0):
        """Spin our node with a fresh executor to drain callbacks."""
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(self)
        deadline = time.time() + timeout
        while time.time() < deadline:
            ex.spin_once(timeout_sec=0.1)
        ex.remove_node(self)

    def _wait_for_amcl(self, timeout=5.0):
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(self)
        deadline = time.time() + timeout
        while self._current_x is None and time.time() < deadline:
            ex.spin_once(timeout_sec=0.1)
        ex.remove_node(self)

    def _switch_to_navigation_mode(self):
        if not self._nav_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('switch_to_navigation_mode service not available.')
            return
        future = self._nav_mode_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result and result.success:
            self.get_logger().info('Switched to navigation mode.')
        else:
            self.get_logger().warn(f'Mode switch response: {result.message if result else "no response"}')

    # ------------------------------------------------------------------ tolerances

    def _set_nav2_tolerances(self):
        client = self.create_client(SetParameters, '/controller_server/set_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('controller_server set_parameters not available.')
            return

        def make_param(name, value, ptype):
            p = RclParameter()
            p.name = name
            p.value = ParameterValue(type=ptype)
            if ptype == ParameterType.PARAMETER_DOUBLE:
                p.value.double_value = value
            return p

        req = SetParameters.Request()
        req.parameters = [
            make_param('FollowPath.xy_goal_tolerance',  XY_TOLERANCE_M,                    ParameterType.PARAMETER_DOUBLE),
            make_param('FollowPath.yaw_goal_tolerance', math.radians(YAW_TOLERANCE_DEG),   ParameterType.PARAMETER_DOUBLE),
        ]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Nav2 tolerances set: xy={XY_TOLERANCE_M*100:.0f} cm, yaw={YAW_TOLERANCE_DEG}°')

    # ------------------------------------------------------------------ initial pose

    def set_initial_pose(self, x, y, yaw_deg=0.0):
        r = math.radians(yaw_deg) / 2
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation.z = math.sin(r)
        pose.pose.pose.orientation.w = math.cos(r)
        pose.pose.covariance[0]  = 0.25
        pose.pose.covariance[7]  = 0.25
        pose.pose.covariance[35] = 0.07
        time.sleep(1.0)
        self._initial_pose_pub.publish(pose)
        self.get_logger().info(f'Initial pose set: x={x}, y={y}, yaw={yaw_deg}°')
        time.sleep(1.0)

    # ------------------------------------------------------------------ main goal

    def send_goal(self, x, y, goal_yaw_deg=0.0):
        # Always switch to navigation mode before doing anything
        self._switch_to_navigation_mode()

        self.get_logger().info('Waiting for navigation server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available.')
            return False

        self._set_nav2_tolerances()

        # Warm up AMCL subscription
        self._wait_for_amcl(timeout=3.0)

        r = math.radians(goal_yaw_deg) / 2
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(r)
        goal.pose.pose.orientation.w = math.cos(r)

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={goal_yaw_deg}°')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Position reached, verifying pose...')
            return self._verify_and_correct(x, y, goal_yaw_deg)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted — robot may be stuck or goal unreachable.')
        else:
            self.get_logger().error(f'Navigation failed, status: {status}')
        return False

    # ------------------------------------------------------------------ verify & correct

    def _verify_and_correct(self, goal_x, goal_y, goal_yaw_deg):
        # Re-switch to navigation mode in case it dropped after translation
        self._switch_to_navigation_mode()

        # Collect fresh AMCL pose; fall back to last known if robot is stationary
        snapshot_x, snapshot_y = self._current_x, self._current_y
        self._wait_for_amcl(timeout=5.0)

        if self._current_x is None:
            self.get_logger().error('Could not read current pose from /amcl_pose.')
            return False

        if self._current_x == snapshot_x and self._current_y == snapshot_y:
            self.get_logger().warn('AMCL silent — using last known pose.')

        dx, dy = self._current_x - goal_x, self._current_y - goal_y
        xy_err  = math.sqrt(dx**2 + dy**2)
        yaw_err = abs(self._angle_diff(goal_yaw_deg, self._current_yaw))

        self.get_logger().info(f'Final pose  x={self._current_x:.4f} y={self._current_y:.4f} yaw={self._current_yaw:.2f}°')
        self.get_logger().info(f'Errors      xy={xy_err*100:.1f} cm  yaw={yaw_err:.2f}°')

        if xy_err > XY_TOLERANCE_M:
            self.get_logger().warn(f'Position error {xy_err*100:.1f} cm exceeds {XY_TOLERANCE_M*100:.0f} cm tolerance.')
            return False

        if yaw_err > YAW_TOLERANCE_DEG:
            return self._spin_to_heading(goal_yaw_deg)

        self.get_logger().info('Goal reached within tolerances!')
        return True

    # ------------------------------------------------------------------ spin

    def _spin_to_heading(self, target_yaw_deg):
        # Ensure navigation mode before spinning
        self._switch_to_navigation_mode()

        if not self._spin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Spin action server not available.')
            return False

        diff = self._angle_diff(target_yaw_deg, self._current_yaw)
        self.get_logger().info(f'Spinning {diff:.2f}° to reach target heading...')

        spin_goal = Spin.Goal()
        spin_goal.target_yaw = math.radians(diff)
        future = self._spin_client.send_goal_async(spin_goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Spin goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._spin_executor(timeout=1.0)  # drain AMCL callbacks
            final_err = abs(self._angle_diff(target_yaw_deg, self._current_yaw))
            self.get_logger().info(f'Final yaw: {self._current_yaw:.2f}°  Error: {final_err:.2f}°')
            if final_err <= YAW_TOLERANCE_DEG:
                self.get_logger().info('Goal reached with correct heading!')
                return True
            else:
                self.get_logger().warn(f'Heading error {final_err:.2f}° still exceeds {YAW_TOLERANCE_DEG}° tolerance.')
                return False
        else:
            self.get_logger().error(f'Spin failed, status: {status}')
            return False


# ------------------------------------------------------------------ entry point

def parse_args():
    parser = argparse.ArgumentParser(description='Send a navigation goal to Stretch.')
    parser.add_argument('--goal-x',   type=float, required=True)
    parser.add_argument('--goal-y',   type=float, required=True)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument('--init-x',   type=float, default=None)
    parser.add_argument('--init-y',   type=float, default=None)
    parser.add_argument('--init-yaw', type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    navigator = Navigator()

    if args.init_x is not None and args.init_y is not None:
        navigator.set_initial_pose(x=args.init_x, y=args.init_y, yaw_deg=args.init_yaw)
    else:
        navigator.get_logger().info('No initial pose provided, assuming AMCL already localized.')

    success = navigator.send_goal(x=args.goal_x, y=args.goal_y, goal_yaw_deg=args.goal_yaw)
    navigator.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
