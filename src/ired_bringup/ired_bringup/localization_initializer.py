import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizationInitializer(Node):
    def __init__(self):
        super().__init__("localization_initializer")

        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_scan_points", 120)
        self.declare_parameter("occupied_tolerance", 0.06)
        self.declare_parameter("minimum_match_score", 0.30)
        self.declare_parameter("minimum_score_margin", 0.04)

        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.max_scan_points = int(self.get_parameter("max_scan_points").value)
        self.occupied_tolerance = float(self.get_parameter("occupied_tolerance").value)
        self.minimum_match_score = float(
            self.get_parameter("minimum_match_score").value
        )
        self.minimum_score_margin = float(
            self.get_parameter("minimum_score_margin").value
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_data = None
        self.map_info = None
        self.scan = None
        self.team_color = None

        self.initial_pose = None
        self.initialpose_attempts = 0
        self.last_initialpose_publish = None

        self.last_amcl_pose = None
        self.last_amcl_time = None
        self.stable_pose_count = 0
        self.ready = False
        self.match_started = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.ready_pub = self.create_publisher(Bool, "/ired/localization/ready", 10)

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(
            LaserScan,
            "/scan_filtered",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/ired_refbox/parsed/team_color",
            self.team_color_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            10,
        )

        self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(
            "Waiting for map, filtered LiDAR scan, team color, and TF."
        )

    def map_callback(self, msg):
        if msg.info.width == 0 or msg.info.height == 0:
            return

        data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )

        occupied = data >= 65
        radius = max(
            1,
            math.ceil(self.occupied_tolerance / msg.info.resolution),
        )

        # Expand occupied cells to tolerate small scan/map discrepancies.
        expanded = occupied.copy()
        height, width = occupied.shape

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue

                src_x0 = max(0, -dx)
                src_x1 = min(width, width - dx)
                src_y0 = max(0, -dy)
                src_y1 = min(height, height - dy)

                dst_x0 = max(0, dx)
                dst_x1 = min(width, width + dx)
                dst_y0 = max(0, dy)
                dst_y1 = min(height, height + dy)

                expanded[dst_y0:dst_y1, dst_x0:dst_x1] |= occupied[
                    src_y0:src_y1, src_x0:src_x1
                ]

        self.map_data = expanded
        self.map_info = msg.info

    def scan_callback(self, msg):
        self.scan = msg

    def team_color_callback(self, msg):
        color = msg.data.strip().upper()
        if color in ("CYAN", "MAGENTA"):
            self.team_color = color

    @staticmethod
    def quaternion_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def scan_points_in_base(self):
        scan = self.scan

        if scan is None or not scan.ranges:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        indices = np.linspace(
            0,
            len(ranges) - 1,
            min(self.max_scan_points, len(ranges)),
            dtype=int,
        )

        ranges = ranges[indices]
        angles = scan.angle_min + indices * scan.angle_increment

        valid = (
            np.isfinite(ranges)
            & (ranges >= max(scan.range_min, 0.10))
            & (ranges < scan.range_max - 0.05)
        )

        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) < 20:
            return None

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self.quaternion_yaw(rotation)

        c = math.cos(yaw)
        s = math.sin(yaw)

        base_x = translation.x + c * x - s * y
        base_y = translation.y + s * x + c * y

        return np.column_stack((base_x, base_y))

    @staticmethod
    def angle_difference(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def score_pose(self, points, x, y, yaw):
        info = self.map_info
        c = math.cos(yaw)
        s = math.sin(yaw)

        world_x = x + c * points[:, 0] - s * points[:, 1]
        world_y = y + s * points[:, 0] + c * points[:, 1]

        origin = info.origin.position
        resolution = info.resolution

        columns = np.floor((world_x - origin.x) / resolution).astype(int)
        rows = np.floor((world_y - origin.y) / resolution).astype(int)

        height, width = self.map_data.shape

        inside = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)

        hits = np.zeros(len(points), dtype=bool)
        hits[inside] = self.map_data[rows[inside], columns[inside]]

        return float(np.mean(hits))

    def candidate_positions(self):
        if self.team_color == "CYAN":
            x_min, x_max = 2.25, 4.75
        else:
            x_min, x_max = -4.75, -2.25

        xs = np.arange(x_min, x_max + 0.001, 0.10)
        ys = np.arange(0.25, 0.751, 0.05)
        yaws = np.arange(0.0, 2.0 * math.pi, math.radians(10.0))

        return xs, ys, yaws

    def find_initial_pose(self, points):
        xs, ys, yaws = self.candidate_positions()
        coarse = []

        for x in xs:
            for y in ys:
                for yaw in yaws:
                    score = self.score_pose(points, x, y, yaw)
                    coarse.append((score, x, y, yaw))

        coarse.sort(reverse=True, key=lambda item: item[0])
        best_coarse = coarse[0]

        # Reject a strong competing match at a distinct position/orientation.
        distinct_scores = []
        for score, x, y, yaw in coarse[1:]:
            position_distance = math.hypot(x - best_coarse[1], y - best_coarse[2])
            yaw_distance = abs(self.angle_difference(yaw, best_coarse[3]))

            if position_distance >= 0.35 or yaw_distance >= math.radians(30):
                distinct_scores.append(score)

        second_score = max(distinct_scores, default=0.0)

        # Refine the best coarse candidate at finer resolution.
        _, cx, cy, cyaw = best_coarse
        best = best_coarse

        for x in np.arange(cx - 0.12, cx + 0.121, 0.025):
            for y in np.arange(cy - 0.12, cy + 0.121, 0.025):
                if not (0.23 <= y <= 0.77):
                    continue

                if self.team_color == "CYAN":
                    if not (2.23 <= x <= 4.77):
                        continue
                elif not (-4.77 <= x <= -2.23):
                    continue

                for offset in range(-7, 8):
                    yaw = cyaw + math.radians(offset * 2)
                    yaw = self.angle_difference(yaw, 0.0)
                    score = self.score_pose(points, x, y, yaw)

                    if score > best[0]:
                        best = (score, x, y, yaw)

        score, x, y, yaw = best
        margin = score - second_score

        self.get_logger().info(
            f"Scan match: x={x:.3f}, y={y:.3f}, "
            f"yaw={math.degrees(yaw):.1f} deg, "
            f"score={score:.3f}, margin={margin:.3f}"
        )

        if score < self.minimum_match_score:
            self.get_logger().error(
                "Localization rejected: scan match score is too low."
            )
            return None

        if margin < self.minimum_score_margin:
            self.get_logger().error(
                "Localization rejected: another starting pose has a "
                "similar scan match score. Check the map and scan."
            )
            return None

        return x, y, yaw

    def publish_initial_pose(self):
        x, y, yaw = self.initial_pose

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Initial uncertainty: 5 cm in x/y and about 10 degrees in yaw.
        msg.pose.covariance[0] = 0.0025
        msg.pose.covariance[7] = 0.0025
        msg.pose.covariance[35] = math.radians(10.0) ** 2

        self.initialpose_pub.publish(msg)
        self.initialpose_attempts += 1
        self.last_initialpose_publish = self.get_clock().now()

        self.get_logger().info(
            f"Published AMCL initial pose " f"({self.initialpose_attempts}/5)."
        )

    def amcl_pose_callback(self, msg):
        now = self.get_clock().now()
        pose = msg.pose.pose
        covariance = msg.pose.covariance

        covariance_ok = (
            covariance[0] < 0.04 and covariance[7] < 0.04 and covariance[35] < 0.12
        )

        current = (
            pose.position.x,
            pose.position.y,
            self.quaternion_yaw(pose.orientation),
        )

        stable = False
        if self.last_amcl_pose is not None:
            old = self.last_amcl_pose
            distance = math.hypot(current[0] - old[0], current[1] - old[1])
            yaw_change = abs(self.angle_difference(current[2], old[2]))
            stable = distance < 0.08 and yaw_change < math.radians(8.0)

        if covariance_ok and stable:
            self.stable_pose_count += 1
        else:
            self.stable_pose_count = 0

        self.last_amcl_pose = current
        self.last_amcl_time = now

        if self.stable_pose_count >= 5 and not self.ready:
            self.ready = True
            self.get_logger().info("AMCL pose is stable. Localization is ready.")

    def timer_callback(self):
        ready_msg = Bool()
        ready_msg.data = self.ready
        self.ready_pub.publish(ready_msg)

        now = self.get_clock().now()

        if self.ready:
            if (
                self.last_amcl_time is None
                or (now - self.last_amcl_time).nanoseconds > 2_000_000_000
            ):
                self.ready = False
                self.stable_pose_count = 0
                self.get_logger().error(
                    "AMCL pose became stale. Movement must remain locked."
                )
            return

        if not self.match_started:
            if self.map_data is None or self.scan is None or self.team_color is None:
                return

            points = self.scan_points_in_base()
            if points is None:
                return

            self.match_started = True
            self.get_logger().info(
                f"Searching the {self.team_color} starting zones "
                "using the stationary LiDAR scan."
            )

            self.initial_pose = self.find_initial_pose(points)

            if self.initial_pose is None:
                self.get_logger().fatal(
                    "Automatic localization failed. Motors will remain locked."
                )
                return

            self.publish_initial_pose()
            return

        if self.initialpose_attempts < 5:
            if (
                self.last_initialpose_publish is None
                or (now - self.last_initialpose_publish).nanoseconds >= 1_000_000_000
            ):
                self.publish_initial_pose()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationInitializer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
