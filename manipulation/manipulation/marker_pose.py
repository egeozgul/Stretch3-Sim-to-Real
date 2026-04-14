#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hello_helpers.hello_misc import HelloNode
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation
import time
import math
import os

# ── Offset from marker center in CAMERA optical frame (metres) ───────────────
# camera_color_optical_frame axes:
#   +X = right in image
#   +Y = down in image
#   +Z = into the scene (depth)
MARKER_OFFSET_CAM_X = 0.0   # right/left in image
MARKER_OFFSET_CAM_Y = 0.0   # down/up in image
MARKER_OFFSET_CAM_Z = 0.0   # closer/further from camera
# ─────────────────────────────────────────────────────────────────────────────

class ArUcoBaseFrameEstimator(HelloNode):
    def __init__(self):
        super().__init__()
        super().main('aruco_base_frame_estimator', 'aruco_base_frame_estimator', wait_for_first_pointcloud=False)

        self.get_logger().info("Moving head camera to detection orientation...")
        self.move_to_pose({
            'joint_head_pan':  -1.675,
            'joint_head_tilt': -0.560,
        }, blocking=True)
        self.get_logger().info("Head camera at detection pose — starting ArUco detection.")

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_pos = None
        self.camera_rot = None
        self._init_camera_pose()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.target_pub = self.create_publisher(PointStamped, '/aruco/target_base_link', 10)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self.camera_matrix = np.array([
            [912.490478515625, 0.0, 643.6722412109375],
            [0.0, 912.8123779296875, 380.4475402832031],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)

        self.marker_size = 0.03
        h = self.marker_size / 2
        self.obj_points = np.array([
            [-h,  h, 0],
            [ h,  h, 0],
            [ h, -h, 0],
            [-h, -h, 0]
        ], dtype=np.float32)

        self.target_marker_id = 202
        self.last_save_time = time.time()
        self.frame_count = 0
        self.detection_count = 0

        os.makedirs("base_detections", exist_ok=True)

        self.get_logger().info("="*60)
        self.get_logger().info("ArUco Base Frame Estimator Started")
        self.get_logger().info(f"Target Marker ID: {self.target_marker_id}")
        self.get_logger().info(f"Marker size: {self.marker_size*100:.1f} cm")
        self.get_logger().info(f"Marker offset (cam frame): X={MARKER_OFFSET_CAM_X}  Y={MARKER_OFFSET_CAM_Y}  Z={MARKER_OFFSET_CAM_Z}")
        self.get_logger().info("Publishing target on: /aruco/target_base_link")
        self.get_logger().info("="*60)

    def _init_camera_pose(self):
        self.get_logger().info("Waiting for camera TF (base_link -> camera_color_optical_frame)...")
        for attempt in range(20):
            try:
                t = self.tf_buffer.lookup_transform(
                    'base_link',
                    'camera_color_optical_frame',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                p = t.transform.translation
                q = t.transform.rotation
                rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
                self.camera_pos = (p.x, p.y, p.z)
                self.camera_rot = rpy
                self.get_logger().info("Camera pose acquired:")
                self.get_logger().info(f"  Position  : X={p.x:+.4f}  Y={p.y:+.4f}  Z={p.z:+.4f}  (m)")
                self.get_logger().info(f"  RPY (deg) : R={rpy[0]:+.2f}  P={rpy[1]:+.2f}  Y={rpy[2]:+.2f}")
                return
            except Exception as e:
                self.get_logger().warn(f"TF attempt {attempt+1}/20 failed: {e}")
                time.sleep(0.5)
        self.get_logger().error("Could not get camera TF after 20 attempts — proceeding without it")

    def estimate_pose(self, corner):
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            corner[0],
            self.camera_matrix,
            self.dist_coeffs
        )
        if not success:
            return None, None
        return rvec, tvec

    def broadcast_marker_frame(self, rvec, tvec):
        # ── Apply offset in camera optical frame BEFORE broadcasting ─────────
        tvec_offset = tvec.copy()
        tvec_offset[0] += MARKER_OFFSET_CAM_X
        tvec_offset[1] += MARKER_OFFSET_CAM_Y
        tvec_offset[2] += MARKER_OFFSET_CAM_Z
        # ─────────────────────────────────────────────────────────────────────

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = 'fruit_marker'
        t.transform.translation.x = float(tvec_offset[0])
        t.transform.translation.y = float(tvec_offset[1])
        t.transform.translation.z = float(tvec_offset[2])
        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = Rotation.from_matrix(rot_mat).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

    def get_marker_in_base_link(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'fruit_marker',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None, None, None

    def publish_target(self, bx, by, bz):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = bx
        msg.point.y = by
        msg.point.z = bz
        self.target_pub.publish(msg)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            display_image = cv_image.copy()

            if ids is not None and self.target_marker_id in ids:
                idx = np.where(ids == self.target_marker_id)[0][0]
                corner = corners[idx]

                rvec, tvec = self.estimate_pose(corner)
                if rvec is None:
                    self.get_logger().warn("Pose estimation failed")
                    return

                # raw camera-frame coords (for display only)
                cx, cy, cz = float(tvec[0]), float(tvec[1]), float(tvec[2])
                cam_distance = math.sqrt(cx**2 + cy**2 + cz**2)

                # broadcast offset tvec → TF → base_link
                self.broadcast_marker_frame(rvec, tvec)
                bx, by, bz = self.get_marker_in_base_link()

                if bx is not None:
                    self.publish_target(bx, by, bz)

                cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs,
                                 rvec, tvec, 0.03)

                center_x = int(np.mean(corner[0][:, 0]))
                center_y = int(np.mean(corner[0][:, 1]))
                self.draw_info(display_image, center_x, center_y,
                               cx, cy, cz, cam_distance, bx, by, bz)

                cv2.putText(display_image, f"MARKER {self.target_marker_id} DETECTED",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                self.detection_count += 1
                self.frame_count += 1

                if self.frame_count % 30 == 0:
                    self.get_logger().info(f"\n{'='*60}")
                    self.get_logger().info(f"✓ MARKER {self.target_marker_id}")
                    self.get_logger().info(f"  [CAMERA raw] X={cx:+.3f}m  Y={cy:+.3f}m  Z={cz:+.3f}m  dist={cam_distance:.3f}m")
                    if bx is not None:
                        self.get_logger().info(f"  [BASE_LINK+offset] X={bx:+.3f}m  Y={by:+.3f}m  Z={bz:+.3f}m  → published")
                    self.get_logger().info(f"{'='*60}")

            else:
                cv2.putText(display_image, f"Looking for marker {self.target_marker_id}...",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(display_image, f"Detections: {self.detection_count}",
                       (10, display_image.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            current_time = time.time()
            if current_time - self.last_save_time > 3.0:
                filename = f"base_detections/frame_{int(current_time)}.jpg"
                cv2.imwrite(filename, display_image)
                self.get_logger().info(f"Saved: {filename}")
                self.last_save_time = current_time

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def draw_info(self, image, cx, cy, x_cam, y_cam, z_cam, dist, bx, by, bz):
        lines = [
            "-- CAMERA FRAME (raw) --",
            f"X: {x_cam:+.3f}m",
            f"Y: {y_cam:+.3f}m",
            f"Z: {z_cam:+.3f}m",
            f"Dist: {dist:.3f}m",
            "-- BASE_LINK+offset --",
        ]
        if bx is not None:
            lines += [f"X: {bx:+.3f}m", f"Y: {by:+.3f}m", f"Z: {bz:+.3f}m"]
        else:
            lines.append("TF not available yet")

        if self.camera_pos is not None:
            px2, py2 = 10, image.shape[0] - 90
            cv2.rectangle(image, (px2, py2), (px2 + 280, py2 + 80), (0, 0, 0), -1)
            cv2.rectangle(image, (px2, py2), (px2 + 280, py2 + 80), (255, 200, 0), 1)
            cv2.putText(image, "-- CAM POSE (init) --",
                        (px2+5, py2+18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
            cv2.putText(image, f"Pos: ({self.camera_pos[0]:+.3f}, {self.camera_pos[1]:+.3f}, {self.camera_pos[2]:+.3f})",
                        (px2+5, py2+38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)
            cv2.putText(image, f"RPY: ({self.camera_rot[0]:+.1f}, {self.camera_rot[1]:+.1f}, {self.camera_rot[2]:+.1f}) deg",
                        (px2+5, py2+58), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)

        box_w, box_h = 220, len(lines) * 25 + 10
        px, py = cx + 10, cy - 15
        cv2.rectangle(image, (px, py), (px + box_w, py + box_h), (0, 0, 0), -1)
        cv2.rectangle(image, (px, py), (px + box_w, py + box_h), (0, 255, 0), 2)
        for i, line in enumerate(lines):
            color = (0, 200, 255) if "BASE" in line or "CAMERA" in line else (0, 255, 0)
            cv2.putText(image, line, (px + 10, py + 20 + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.drawMarker(image, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)


def main(args=None):
    node = ArUcoBaseFrameEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()