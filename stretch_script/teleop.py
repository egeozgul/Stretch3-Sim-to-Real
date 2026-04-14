#!/usr/bin/env python3
"""
keyboard_teleop.py  —  Drive Stretch with WASD / arrow keys
Run while launch_nodes.sh is active:
    python3 ~/stretch_script/keyboard_teleop.py
"""

import sys
import tty
import termios
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# ── Tunable speeds ─────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.2   # m/s
ANGULAR_SPEED = 0.6   # rad/s

BINDINGS = {
    # key : (linear_x, angular_z)
    'w':        ( 1,  0),
    's':        (-1,  0),
    'a':        ( 0,  1),
    'd':        ( 0, -1),
    '\x1b[A':   ( 1,  0),   # arrow up
    '\x1b[B':   (-1,  0),   # arrow down
    '\x1b[D':   ( 0,  1),   # arrow left
    '\x1b[C':   ( 0, -1),   # arrow right
}

HELP = """
┌─────────────────────────────────┐
│   Stretch Keyboard Teleop       │
│                                 │
│     W / ↑  : forward            │
│     S / ↓  : backward           │
│     A / ←  : turn left          │
│     D / →  : turn right         │
│     space  : stop               │
│     q      : quit               │
│                                 │
│  Linear  speed: {lin:.2f} m/s      │
│  Angular speed: {ang:.2f} rad/s    │
└─────────────────────────────────┘
""".format(lin=LINEAR_SPEED, ang=ANGULAR_SPEED)


def get_key(settings):
    """Read one keypress (handles 3-byte arrow escape sequences)."""
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        ch2 = sys.stdin.read(1)
        ch3 = sys.stdin.read(1)
        ch = ch + ch2 + ch3
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        #self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)        
        self._stop_event = threading.Event()

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def ensure_navigation_mode(self):
        self.get_logger().info("Switching to navigation mode...")
        client = self.create_client(Trigger, '/switch_to_navigation_mode')
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("switch_to_navigation_mode service not available!")
            return
        client.call_async(Trigger.Request())
        time.sleep(1.0)
        self.get_logger().info("Navigation mode ready.")

    def run(self):
        self.ensure_navigation_mode()
        settings = termios.tcgetattr(sys.stdin)
        print(HELP)
        try:
            while not self._stop_event.is_set():
                key = get_key(settings)

                if key == 'q' or key == '\x03':   # q or Ctrl-C
                    print("\nQuitting — stopping robot.")
                    self.stop()
                    self._stop_event.set()
                    break
                elif key == ' ':
                    print("STOP")
                    self.stop()
                elif key in BINDINGS:
                    lx, az = BINDINGS[key]
                    self.publish(lx * LINEAR_SPEED, az * ANGULAR_SPEED)
                # unknown keys are silently ignored
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = KeyboardTeleop()

    # Spin ROS in a background thread so the main thread can block on keyboard
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
