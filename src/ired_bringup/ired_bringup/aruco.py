#!/usr/bin/env python3

import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .camera import Camera

DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Parameters
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("warmup_frames", 3)

        self.declare_parameter("marker_size_m", 0.07)
        self.declare_parameter("dictionary", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("mode", "all")

        # Camera intrinsics
        self.declare_parameter("fx", 600.0)
        self.declare_parameter("fy", 600.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

        # Get parameters
        camera_index = self.get_parameter("camera_index").value
        frame_width = self.get_parameter("frame_width").value
        frame_height = self.get_parameter("frame_height").value
        warmup_frames = self.get_parameter("warmup_frames").value

        self.marker_size = self.get_parameter("marker_size_m").value

        dictionary_name = self.get_parameter("dictionary").value
        self.mode = self.get_parameter("mode").value

        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value

        # Camera
        self.camera = Camera(
            index=camera_index,
            width=frame_width,
            height=frame_height,
            warmup_frames=warmup_frames,
        )

        # Camera calibration
        self.camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        # TODO: replace with actual camera distortion parameters
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # ArUco
        if dictionary_name not in DICTIONARIES:
            self.get_logger().error(f"Unknown ArUco dictionary: {dictionary_name}")
            self.get_logger().error(f"Available dictionaries: {list(DICTIONARIES)}")
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")

        dictionary_id = DICTIONARIES[dictionary_name]

        # OpenCV 4.6 legacy API
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Marker corners in marker-local coordinates
        half = self.marker_size / 2.0

        self.marker_object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

        # ROS
        self.bridge = CvBridge()

        self.scan_service = self.create_service(
            Trigger,
            "/aruco/scan",
            self.scan_callback,
        )

        self.result_pub = self.create_publisher(
            String,
            "/aruco/detections",
            10,
        )

        self.image_pub = self.create_publisher(
            Image,
            "/aruco/image",
            10,
        )

        self.get_logger().info(
            f"ArUco node ready " f"(dictionary={dictionary_name}, mode={self.mode})"
        )

    def scan_callback(self, request, response):
        del request

        # Capture one still frame.
        try:
            frame = self.camera.capture()
        except RuntimeError as e:
            response.success = False
            response.message = str(e)
            return response

        # Detect markers and annotate the captured frame.
        detections = self.detect(frame)

        if self.mode == "closest" and detections:
            detections = detections[:1]

        # Publish the exact frame used for detection.
        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_link"

        self.image_pub.publish(image_msg)

        # Publish detection result.
        result = {
            "count": len(detections),
            "detections": detections,
        }

        message = json.dumps(result)

        result_msg = String()
        result_msg.data = message

        self.result_pub.publish(result_msg)

        response.success = True
        response.message = message

        self.get_logger().info(f"ArUco scan: {message}")

        return response

    def detect(self, frame):
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY,
        )

        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

        if ids is None or len(ids) == 0:
            return []

        # Draw all detected markers on the image.
        cv2.aruco.drawDetectedMarkers(
            frame,
            corners,
            ids,
        )

        detections = []

        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = np.ascontiguousarray(
                corners[i][0],
                dtype=np.float64,
            )

            # Calculate apparent marker size in pixels.
            edge_lengths = []

            for j in range(4):
                p1 = marker_corners[j]
                p2 = marker_corners[(j + 1) % 4]

                edge_lengths.append(np.linalg.norm(p2 - p1))

            size_px = float(np.mean(edge_lengths))

            # Estimate marker pose.
            ok, rvec, tvec = cv2.solvePnP(
                self.marker_object_points,
                marker_corners,
                self.camera_matrix,
                self.dist_coeffs,
            )

            if not ok:
                continue

            x = float(tvec[0][0])
            y = float(tvec[1][0])
            z = float(tvec[2][0])

            distance = math.sqrt(x * x + y * y + z * z)

            # Calculate apparent angular size.
            angular_size_rad = 2.0 * math.atan(
                size_px / (2.0 * self.camera_matrix[0, 0])
            )

            angular_size_deg = math.degrees(angular_size_rad)

            # Draw the estimated pose axes.
            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_size * 0.5,
            )

            # Draw useful information next to the marker.
            center = np.mean(marker_corners, axis=0)
            center_x = int(center[0])
            center_y = int(center[1])

            label = (
                f"ID {int(marker_id)} " f"{distance:.2f}m " f"{angular_size_deg:.1f}deg"
            )

            cv2.putText(
                frame,
                label,
                (center_x, center_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            detections.append(
                {
                    "id": int(marker_id),
                    "distance_m": distance,
                    "size_px": size_px,
                    "angular_size_deg": angular_size_deg,
                    "position_m": {
                        "x": x,
                        "y": y,
                        "z": z,
                    },
                }
            )

        # Closest first.
        detections.sort(key=lambda detection: detection["distance_m"])

        return detections


def main(args=None):
    rclpy.init(args=args)

    node = ArucoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
