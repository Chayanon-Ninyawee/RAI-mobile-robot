import json
import math
import re

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def parse_zone(zone):
    if not isinstance(zone, str):
        return None

    match = re.fullmatch(r"[CM]_Z([1-5])([1-5])", zone.strip())

    if not match:
        return None

    column = int(match.group(1))
    row = int(match.group(2))

    x = column - 0.5
    y = row - 0.5

    if zone.startswith("M_"):
        x = -x

    return x, y


def distance(first, second):
    return math.hypot(
        first[0] - second[0],
        first[1] - second[1],
    )


class NavigationChallenge(Node):
    def __init__(self):
        super().__init__("navigation_challenge")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("initial_point_wait", 2.0)
        self.declare_parameter("retry_delay", 2.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.initial_point_wait = float(self.get_parameter("initial_point_wait").value)
        self.retry_delay = float(self.get_parameter("retry_delay").value)

        self.team_color = "CYAN"
        self.game_state = {}
        self.routes_data = {}
        self.machines_data = {}

        self.route_zones = []
        self.remaining_zones = []
        self.visited_zones = set()

        self.initial_zone = None
        self.current_zone = None
        self.initial_wait_until = None

        self.goal_handle = None
        self.goal_pending = False
        self.active_goal = None
        self.retry_after = None

        self.challenge_complete = False
        self.stopping = False

        self.create_subscription(
            String,
            "/ired_refbox/parsed/gamestate",
            self.gamestate_callback,
            10,
        )

        self.create_subscription(
            String,
            "/ired_refbox/parsed/navigationroutes",
            self.routes_callback,
            10,
        )

        self.create_subscription(
            String,
            "/ired_refbox/parsed/machineinfo",
            self.machines_callback,
            10,
        )

        self.create_subscription(
            String,
            "/ired_refbox/parsed/team_color",
            self.team_color_callback,
            10,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self.status_pub = self.create_publisher(
            String,
            "/ired/navigation_challenge/status",
            10,
        )

        self.machine_marker_pub = self.create_publisher(
            MarkerArray,
            "/ired/navigation_challenge/machine_markers",
            10,
        )

        self.create_timer(0.2, self.update)

        self.get_logger().info("Navigation challenge ready")

    # ---------------------------------------------------------
    # RefBox subscriptions
    # ---------------------------------------------------------

    def gamestate_callback(self, msg):
        try:
            self.game_state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.game_state = {}

    def routes_callback(self, msg):
        try:
            self.routes_data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.routes_data = {}

    def machines_callback(self, msg):
        try:
            self.machines_data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.machines_data = {}

    def team_color_callback(self, msg):
        color = msg.data.strip().upper()

        if color in ("CYAN", "MAGENTA"):
            self.team_color = color

    # ---------------------------------------------------------
    # Challenge state
    # ---------------------------------------------------------

    def is_active(self):
        return (
            self.game_state.get("state") == "RUNNING"
            and self.game_state.get("phase") == "PRODUCTION"
            and self.game_state.get("challenge_type") == "NAVIGATION"
        )

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def stop(self):
        self.stopping = True

        if self.goal_handle is not None:
            self.get_logger().info("Cancelling active Nav2 goal")

            self.goal_handle.cancel_goal_async()

        self.publish_status("INACTIVE")

    # ---------------------------------------------------------
    # Route selection
    # ---------------------------------------------------------

    def select_route(self):
        routes_data = self.routes_data
        routes = routes_data.get("routes", [])

        if not isinstance(routes, list) or not routes:
            return []

        route_color = str(routes_data.get("team_color", "")).upper()

        if route_color and route_color != self.team_color:
            return []

        # Your parser puts team_color at the top level.
        # If there is one route, use it directly.
        if len(routes) == 1:
            selected = routes[0]
        else:
            # If multiple routes are supplied, use an explicit
            # per-route team_color when available.
            selected = next(
                (
                    route
                    for route in routes
                    if isinstance(route, dict)
                    and str(route.get("team_color", "")).upper() == self.team_color
                ),
                None,
            )

        if not isinstance(selected, dict):
            self.get_logger().warn(f"No route found for team {self.team_color}")
            return []

        remaining = selected.get("remaining", [])
        route = selected.get("route", [])

        zones = remaining if remaining else route

        if isinstance(zones, str):
            zones = [zones]

        return [
            zone.strip()
            for zone in zones
            if isinstance(zone, str) and parse_zone(zone) is not None
        ]

    def load_route(self):
        zones = self.select_route()

        if not zones:
            self.publish_status("WAITING_FOR_ROUTE")
            return False

        # The first point is mandatory. Only reorder the rest.
        self.initial_zone = zones[0]
        self.route_zones = self.optimize_route(zones)
        self.remaining_zones = list(self.route_zones)
        self.visited_zones.clear()

        self.current_zone = None
        self.initial_wait_until = None
        self.challenge_complete = False
        self.retry_after = None

        self.get_logger().info(
            f"Loaded {self.team_color} route: " f"{self.route_zones}"
        )

        self.publish_status("ROUTE_LOADED")
        return True

    def optimize_route(self, zones):
        if len(zones) <= 2:
            return list(zones)

        ordered = [zones[0]]
        unvisited = list(zones[1:])
        current = parse_zone(zones[0])

        while unvisited:
            next_zone = min(
                unvisited,
                key=lambda zone: distance(
                    current,
                    parse_zone(zone),
                ),
            )

            ordered.append(next_zone)
            unvisited.remove(next_zone)
            current = parse_zone(next_zone)

        return ordered

    # ---------------------------------------------------------
    # Nav2 goals
    # ---------------------------------------------------------

    def make_goal(self, zone, next_zone=None):
        position = parse_zone(zone)

        if position is None:
            return None

        x, y = position
        yaw = 0.0

        if next_zone is not None:
            next_position = parse_zone(next_zone)

            if next_position is not None:
                dx = next_position[0] - x
                dy = next_position[1] - y

                if math.hypot(dx, dy) > 0.001:
                    yaw = math.atan2(dy, dx)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()

        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        return goal

    def send_goal(self, zone, next_zone=None):
        if self.stopping or self.goal_pending:
            return

        if self.goal_handle is not None:
            return

        if not self.nav_client.server_is_ready():
            self.publish_status("WAITING_FOR_NAV2")
            return

        goal = self.make_goal(zone, next_zone)

        if goal is None:
            self.get_logger().error(f"Invalid goal zone: {zone}")
            return

        self.goal_pending = True
        self.active_goal = zone

        self.publish_status(f"GOING_TO:{zone}")
        self.get_logger().info(f"Navigating to {zone}")

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_pending = False

        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Nav2 goal request failed: {error}")
            self.schedule_retry()
            return

        if not handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal")
            self.schedule_retry()
            return

        self.goal_handle = handle

        if self.stopping or not self.is_active():
            handle.cancel_goal_async()

        result_future = handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def schedule_retry(self):
        self.active_goal = None
        self.retry_after = self.get_clock().now() + Duration(seconds=self.retry_delay)

    def goal_result_callback(self, future):
        completed_zone = self.active_goal

        self.goal_handle = None
        self.active_goal = None

        if self.stopping or not self.is_active():
            return

        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f"Nav2 result failed: {error}")
            self.schedule_retry()
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"Goal {completed_zone} failed " f"(status {result.status}); retrying"
            )
            self.schedule_retry()
            return

        if completed_zone is None:
            return

        self.get_logger().info(f"Reached {completed_zone}")

        self.visited_zones.add(completed_zone)
        self.remaining_zones = [
            zone for zone in self.remaining_zones if zone != completed_zone
        ]
        self.current_zone = completed_zone
        self.retry_after = None

        if completed_zone == self.initial_zone:
            self.initial_wait_until = self.get_clock().now() + Duration(
                seconds=self.initial_point_wait
            )

        if not self.remaining_zones:
            self.challenge_complete = True
            self.publish_status("COMPLETE")
            self.get_logger().info("All navigation points completed")

    # ---------------------------------------------------------
    # Known MPS visualization
    # ---------------------------------------------------------

    def publish_machine_markers(self):
        machines = self.machines_data.get("machines", [])

        if not isinstance(machines, list):
            return

        markers = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        for index, machine in enumerate(machines):
            if not isinstance(machine, dict):
                continue

            zone = machine.get("zone")

            if not zone:
                continue

            position = parse_zone(zone)

            if position is None:
                continue

            x, y = position

            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "known_mps"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.55
            marker.scale.y = 0.55
            marker.scale.z = 0.5

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.8

            marker.text = str(machine.get("name", zone))
            markers.markers.append(marker)

        self.machine_marker_pub.publish(markers)

    # ---------------------------------------------------------
    # Main state machine
    # ---------------------------------------------------------

    def update(self):
        if self.stopping:
            return

        if not self.is_active():
            return

        self.publish_machine_markers()

        if not self.route_zones:
            if not self.load_route():
                return

        if self.challenge_complete:
            return

        if self.goal_pending or self.goal_handle is not None:
            return

        if self.retry_after is not None:
            if self.get_clock().now() < self.retry_after:
                return

            self.retry_after = None

        # Always visit the first route point before any others.
        if self.initial_zone not in self.visited_zones:
            next_zone = self.route_zones[1] if len(self.route_zones) > 1 else None
            self.send_goal(self.initial_zone, next_zone)
            return

        # Wait only at the initial point.
        if self.initial_wait_until is not None:
            if self.get_clock().now() < self.initial_wait_until:
                return

            self.initial_wait_until = None

        if not self.remaining_zones:
            self.challenge_complete = True
            self.publish_status("COMPLETE")
            return

        current = parse_zone(self.current_zone)

        if current is None:
            current = parse_zone(self.initial_zone)

        next_zone = min(
            self.remaining_zones,
            key=lambda zone: distance(
                current,
                parse_zone(zone),
            ),
        )

        candidates = [zone for zone in self.remaining_zones if zone != next_zone]

        following_zone = None

        if candidates:
            next_position = parse_zone(next_zone)

            following_zone = min(
                candidates,
                key=lambda zone: distance(
                    next_position,
                    parse_zone(zone),
                ),
            )

        self.send_goal(next_zone, following_zone)


def main(args=None):
    rclpy.init(args=args)

    node = NavigationChallenge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
