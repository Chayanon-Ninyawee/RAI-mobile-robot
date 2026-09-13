#!/usr/bin/env python3
"""
ArUco detection + RViz livestream + map-frame marker visualization.

NOTE: this OpenCV build (4.6.0, Ubuntu Noble arm64) does NOT have the newer
cv2.aruco.ArucoDetector class. Mixing the new-style cv2.aruco.DetectorParameters()
object with the old-style cv2.aruco.detectMarkers() function segfaults at the C++
binding layer (confirmed on this hardware). This version uses ONLY the legacy,
function-based API, which is the one that actually works here:

    dictionary = cv2.aruco.getPredefinedDictionary(...)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to a (x, y, z, w) quaternion."""
    trace = np.trace(R)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


class ArucoStreamNode(Node):
    def __init__(self):
        super().__init__('aruco_stream_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('marker_size_m', 0.07)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.marker_size = self.get_parameter('marker_size_m').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        self.camera_matrix = np.array([[fx, 0, cx],
                                        [0, fy, cy],
                                        [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1))

        m = self.marker_size / 2.0
        self.marker_obj_points = np.array([
            [-m,  m, 0],
            [ m,  m, 0],
            [ m, -m, 0],
            [-m, -m, 0],
        ], dtype=np.float64)

        # ---------------- ROS I/O ----------------
        self.bridge = CvBridge()
        self.id_pub = self.create_publisher(Int32MultiArray, 'aruco_ids', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.marker_pub = self.create_publisher(Marker, 'aruco/map_markers', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Camera setup ----------------
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera index {self.camera_index}')

        # ---------------- ArUco setup: LEGACY API ONLY ----------------
        # Confirmed working on this OpenCV 4.6.0 build. Do NOT mix in
        # cv2.aruco.DetectorParameters() (class) or cv2.aruco.ArucoDetector
        # (doesn't exist here) -- that combination segfaults.
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
        self.get_logger().info('ArUco detection, RViz streaming, and map marker publishing active.')

    def publish_camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.camera_frame
        info.width = self.frame_width
        info.height = self.frame_height
        info.distortion_model = 'plumb_bob'
        info.d = self.dist_coeffs.flatten().tolist()
        info.k = self.camera_matrix.flatten().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fx, fy, cx, cy = self.camera_matrix[0, 0], self.camera_matrix[1, 1], \
            self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None or frame.size == 0:
            return

        stamp = self.get_clock().now().to_msg()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters)

        if ids is not None and len(ids) > 0:
            id_msg = Int32MultiArray()
            id_msg.data = [int(i) for i in ids.flatten()]
            self.id_pub.publish(id_msg)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                img_points = np.ascontiguousarray(corners[i][0], dtype=np.float64)
                ok, rvec, tvec = cv2.solvePnP(
                    self.marker_obj_points, img_points,
                    self.camera_matrix, self.dist_coeffs)
                if not ok:
                    continue

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                   rvec, tvec, self.marker_size * 0.5)

                R, _ = cv2.Rodrigues(rvec)
                qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = self.camera_frame
                t.child_frame_id = f'aruco_{int(marker_id)}'
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(t)

                self.publish_map_marker(marker_id, t, stamp)

        self.publish_camera_info(stamp)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.camera_frame
        self.image_pub.publish(img_msg)

    def publish_map_marker(self, marker_id, camera_to_marker_tf, stamp):
        pose_in_camera = PoseStamped()
        pose_in_camera.header.frame_id = self.camera_frame
        pose_in_camera.header.stamp = stamp
        pose_in_camera.pose.position.x = camera_to_marker_tf.transform.translation.x
        pose_in_camera.pose.position.y = camera_to_marker_tf.transform.translation.y
        pose_in_camera.pose.position.z = camera_to_marker_tf.transform.translation.z
        pose_in_camera.pose.orientation = camera_to_marker_tf.transform.rotation

        try:
            pose_in_map = self.tf_buffer.transform(
                pose_in_camera, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'Could not transform marker {marker_id} into "{self.map_frame}": {e}',
                throttle_duration_sec=2.0)
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'aruco_markers'
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_in_map.pose
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
