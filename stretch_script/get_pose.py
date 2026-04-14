import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseGetter(Node):
    def __init__(self):
        super().__init__('pose_getter')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos)
        # Timeout timer — exit with error if no message after 10 seconds
        self.timer = self.create_timer(10.0, self.timeout_cb)

    def pose_callback(self, msg):
        self.timer.cancel()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        x_or=msg.pose.pose.orientation.x
        y_or=msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = math.degrees(2 * math.atan2(z, w))
        print(f'x:   {x:.4f} m')
        print(f'y:   {y:.4f} m')
        print(f'yaw: {yaw:.2f} deg')
        print(f'quat x: {x_or:.8f} y: {y_or:.8f} z: {z:.8f} w: {w:.8f}')
        raise SystemExit(0)

    def timeout_cb(self):
        print('ERROR: No message on /amcl_pose after 5s.')
        print('Make sure navigation.launch.py is running in another terminal.')
        raise SystemExit(1)


def main():
    rclpy.init()
    node = PoseGetter()
    # Spin briefly to let the subscriber connect before any message arrives
    rclpy.spin_once(node, timeout_sec=1.0)
    try:
        rclpy.spin(node)
    except SystemExit as e:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
