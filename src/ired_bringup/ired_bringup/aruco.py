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

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("warmup_seconds", 1.0)

        self.declare_parameter("marker_size_m", 0.07)
        self.declare_parameter("dictionary", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("mode", "all")

        self.declare_parameter("fx", 600.0)
        self.declare_parameter("fy", 600.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

        camera_index = self.get_parameter("camera_index").value
        frame_width = self.get_parameter("frame_width").value
        frame_height = self.get_parameter("frame_height").value
        warmup_seconds = self.get_parameter("warmup_seconds").value

        self.marker_size = self.get_parameter("marker_size_m").value

        dictionary_name = self.get_parameter("dictionary").value
        self.mode = self.get_parameter("mode").value

        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value

        self.camera = Camera(
            index=camera_index,
            width=frame_width,
            height=frame_height,
            warmup_seconds=warmup_seconds,
        )

        self.camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        # TODO: replace with actual camera distortion parameters.
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        if dictionary_name not in DICTIONARIES:
            self.get_logger().error(f"Unknown ArUco dictionary: {dictionary_name}")
            self.get_logger().error(f"Available dictionaries: {list(DICTIONARIES)}")
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")

        dictionary_id = DICTIONARIES[dictionary_name]

        # OpenCV 4.6 legacy API.
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        self.parameters = cv2.aruco.DetectorParameters_create()

        # Marker detection parameters.
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 51
        self.parameters.adaptiveThreshWinSizeStep = 2
        self.parameters.adaptiveThreshConstant = 7

        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.maxMarkerPerimeterRate = 4.0

        self.parameters.polygonalApproxAccuracyRate = 0.05

        self.parameters.minCornerDistanceRate = 0.01
        self.parameters.minDistanceToBorder = 1

        # Marker corners in marker-local coordinates.
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

        try:
            frame = self.camera.capture()
            cv2.imwrite("/tmp/aruco_test.jpg", frame)
        except RuntimeError as e:
            response.success = False
            response.message = str(e)
            return response

        detections = self.detect(frame)

        if self.mode == "closest" and detections:
            detections = detections[:1]

        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_link"

        self.image_pub.publish(image_msg)

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

        # First let OpenCV find candidate quadrilaterals.
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

        self.get_logger().info(
            f"Initial ArUco detection: "
            f"ids={None if ids is None else ids.flatten().tolist()}, "
            f"rejected={len(rejected)}"
        )

        detections = []

        # Keep any markers OpenCV decoded normally.
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                detections.append(
                    self.process_detection(
                        frame,
                        corners[i][0],
                        int(marker_id),
                    )
                )

        # OpenCV 4.6 may find the marker as a rejected candidate
        # but fail to decode it because of perspective/thresholding.
        for rejected_corners in rejected:
            marker_id = self.decode_candidate(
                gray,
                rejected_corners[0],
            )

            if marker_id is None:
                continue

            # Avoid adding the same marker twice.
            if any(detection["id"] == marker_id for detection in detections):
                continue

            marker_corners = rejected_corners[0]

            detections.append(
                self.process_detection(
                    frame,
                    marker_corners,
                    marker_id,
                )
            )

        detections.sort(
            key=lambda detection: detection.get(
                "distance_m",
                float("inf"),
            )
        )

        return detections

    def decode_candidate(self, gray, corners):
        corners = np.asarray(
            corners,
            dtype=np.float32,
        )

        size = 400

        destination = np.array(
            [
                [0, 0],
                [size - 1, 0],
                [size - 1, size - 1],
                [0, size - 1],
            ],
            dtype=np.float32,
        )

        transform = cv2.getPerspectiveTransform(
            corners,
            destination,
        )

        warped = cv2.warpPerspective(
            gray,
            transform,
            (size, size),
        )

        # The normal grayscale image may fail because the marker
        # contrast is not suitable for OpenCV's decoder.
        #
        # Try several fixed thresholds on the rectified candidate.
        for threshold in range(80, 241, 10):
            _, binary = cv2.threshold(
                warped,
                threshold,
                255,
                cv2.THRESH_BINARY,
            )

            candidate_corners, ids, _rejected = cv2.aruco.detectMarkers(
                binary,
                self.dictionary,
                parameters=self.parameters,
            )

            del candidate_corners

            if ids is None:
                continue

            marker_ids = ids.flatten().tolist()

            # Ignore the known false-positive maximum ID that
            # appeared during threshold testing.
            marker_ids = [marker_id for marker_id in marker_ids if marker_id != 1023]

            if not marker_ids:
                continue

            return int(marker_ids[0])

        return None

    def process_detection(
        self,
        frame,
        marker_corners,
        marker_id,
    ):
        marker_corners = np.ascontiguousarray(
            marker_corners,
            dtype=np.float64,
        )

        edge_lengths = []

        for j in range(4):
            p1 = marker_corners[j]
            p2 = marker_corners[(j + 1) % 4]

            edge_lengths.append(np.linalg.norm(p2 - p1))

        size_px = float(np.mean(edge_lengths))

        detection = {
            "id": int(marker_id),
            "size_px": size_px,
        }

        ok, rvec, tvec = cv2.solvePnP(
            self.marker_object_points,
            marker_corners,
            self.camera_matrix,
            self.dist_coeffs,
        )

        if ok:
            x = float(tvec[0][0])
            y = float(tvec[1][0])
            z = float(tvec[2][0])

            distance = math.sqrt(x * x + y * y + z * z)

            angular_size_rad = 2.0 * math.atan(
                size_px / (2.0 * self.camera_matrix[0, 0])
            )

            angular_size_deg = math.degrees(angular_size_rad)

            detection["distance_m"] = distance
            detection["angular_size_deg"] = angular_size_deg
            detection["position_m"] = {
                "x": x,
                "y": y,
                "z": z,
            }

            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_size * 0.5,
            )

        else:
            self.get_logger().warn(f"solvePnP failed for marker ID {marker_id}")

        center = np.mean(
            marker_corners,
            axis=0,
        )

        center_x = int(center[0])
        center_y = int(center[1])

        label = f"ID {marker_id}"

        if "distance_m" in detection:
            label += f" {detection['distance_m']:.2f}m"

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

        return detection


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
