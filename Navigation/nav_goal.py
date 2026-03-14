import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType
YAW_TOLERANCE_DEG = 3.0   # acceptable heading error in degrees
XY_TOLERANCE_M    = 0.05  # acceptable position error in meters (5 cm)


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._current_yaw = None
        self._current_x = None
        self._current_y = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

    def _amcl_cb(self, msg):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self._current_yaw = math.degrees(2 * math.atan2(z, w))

    def _angle_diff(self, a, b):
        """Shortest signed difference between two angles in degrees."""
        return (a - b + 180) % 360 - 180

    def _set_nav2_tolerances(self):
        """Push tighter tolerances into the Nav2 controller server."""
        client = self.create_client(SetParameters, '/controller_server/set_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('controller_server set_parameters not available, using default tolerances.')
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
            make_param('FollowPath.xy_goal_tolerance',  XY_TOLERANCE_M,              ParameterType.PARAMETER_DOUBLE),
            make_param('FollowPath.yaw_goal_tolerance', math.radians(YAW_TOLERANCE_DEG), ParameterType.PARAMETER_DOUBLE),
        ]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(
            f'Nav2 tolerances set: xy={XY_TOLERANCE_M*100:.0f} cm, yaw={YAW_TOLERANCE_DEG}°')

    def set_initial_pose(self, x, y, yaw_deg=0.0):
        r = math.radians(yaw_deg) / 2
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation.z = math.sin(r)
        pose.pose.pose.orientation.w = math.cos(r)
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.07
        time.sleep(1.0)
        self._initial_pose_pub.publish(pose)
        self.get_logger().info(f'Initial pose set: x={x}, y={y}, yaw={yaw_deg}°')
        time.sleep(1.0)

    def send_goal(self, x, y, goal_yaw_deg=0.0):
        self.get_logger().info('Waiting for navigation server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available. Is navigation.launch.py running?')
            return False

        self._set_nav2_tolerances()

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
            self.get_logger().error('Goal rejected by navigation server!')
            return False

        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Position reached, verifying pose...')
            return self._verify_and_correct(x, y, goal_yaw_deg)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation was canceled.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted. Robot may be stuck or goal is unreachable.')
        else:
            self.get_logger().error(f'Navigation failed with status code: {status}')
        return False

    def _verify_and_correct(self, goal_x, goal_y, goal_yaw_deg):
        """Check final x/y and yaw, spin to correct heading if needed."""
        # Wait for fresh amcl_pose
        deadline = time.time() + 5.0
        while (self._current_yaw is None or self._current_x is None) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._current_x is None:
            self.get_logger().error('Could not read current pose from /amcl_pose.')
            return False

        dx = self._current_x - goal_x
        dy = self._current_y - goal_y
        xy_err = math.sqrt(dx**2 + dy**2)
        yaw_err = abs(self._angle_diff(goal_yaw_deg, self._current_yaw))

        self.get_logger().info(
            f'Final pose  x={self._current_x:.4f} y={self._current_y:.4f} yaw={self._current_yaw:.2f}°')
        self.get_logger().info(
            f'Errors      xy={xy_err*100:.1f} cm  yaw={yaw_err:.2f}°')

        # Check position
        if xy_err > XY_TOLERANCE_M:
            self.get_logger().warn(
                f'Position error {xy_err*100:.1f} cm exceeds tolerance {XY_TOLERANCE_M*100:.0f} cm.')
            return False

        # Correct heading if needed
        if yaw_err > YAW_TOLERANCE_DEG:
            return self._spin_to_heading(goal_yaw_deg)

        self.get_logger().info('Goal reached within position and heading tolerances!')
        return True

    def _spin_to_heading(self, target_yaw_deg):
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
            rclpy.spin_once(self, timeout_sec=0.5)
            final_err = abs(self._angle_diff(target_yaw_deg, self._current_yaw))
            self.get_logger().info(f'Final yaw: {self._current_yaw:.2f}°  Error: {final_err:.2f}°')
            if final_err <= YAW_TOLERANCE_DEG:
                self.get_logger().info('Goal reached with correct heading!')
                return True
            else:
                self.get_logger().warn(f'Heading error {final_err:.2f}° still exceeds {YAW_TOLERANCE_DEG}° tolerance.')
                return False
        else:
            self.get_logger().error(f'Spin failed with status: {status}')
            return False


def parse_args():
    parser = argparse.ArgumentParser(description='Send a navigation goal to Stretch.')
    parser.add_argument('--goal-x',   type=float, required=True, help='Goal x in map frame (meters)')
    parser.add_argument('--goal-y',   type=float, required=True, help='Goal y in map frame (meters)')
    parser.add_argument('--goal-yaw', type=float, default=0.0,   help='Goal yaw in degrees (0=+X, 90=+Y, 180=-X, -90=-Y)')
    parser.add_argument('--init-x',   type=float, default=None,  help='Initial pose x (skip if AMCL already localized)')
    parser.add_argument('--init-y',   type=float, default=None,  help='Initial pose y (skip if AMCL already localized)')
    parser.add_argument('--init-yaw', type=float, default=0.0,   help='Initial pose yaw in degrees')
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
