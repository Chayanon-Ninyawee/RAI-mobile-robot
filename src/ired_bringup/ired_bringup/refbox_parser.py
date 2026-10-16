import json
import math
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def parse_blocks(text, field):
    pattern = rf"\b{re.escape(field)}\s*\{{([^{{}}]*)\}}"
    return [match.group(1) for match in re.finditer(pattern, text)]


def parse_fields(text):
    fields = {}

    pattern = r'^\s*([A-Za-z_]\w*):\s*(?:"([^"]*)"|([^\s]+))\s*$'

    for match in re.finditer(pattern, text, re.MULTILINE):
        key = match.group(1)
        value = match.group(2) if match.group(2) is not None else match.group(3)

        if key in fields:
            if not isinstance(fields[key], list):
                fields[key] = [fields[key]]
            fields[key].append(value)
        else:
            fields[key] = value

    # Preserve protobuf numeric fields such as 5: 122 and 6: 121.
    numeric_pattern = r"^\s*(\d+):\s*(-?\d+)\s*$"

    for match in re.finditer(numeric_pattern, text, re.MULTILINE):
        fields[match.group(1)] = int(match.group(2))

    return fields


def parse_zone(zone):
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


def parse_repeated_values(text, field):
    pattern = rf'^\s*{re.escape(field)}:\s*(?:"([^"]*)"|([^\s]+))\s*$'

    return [
        match.group(1) if match.group(1) is not None else match.group(2)
        for match in re.finditer(pattern, text, re.MULTILINE)
    ]


class RefboxParser(Node):
    def __init__(self):
        super().__init__("refbox_parser")

        self.declare_parameter("team_color", "CYAN")
        self.team_color = self.get_parameter("team_color").value

        self.machineinfo = ""
        self.navigationroutes = ""
        self.gamestate = ""

        self.create_subscription(
            String,
            "/ired_refbox/machineinfo",
            self.machineinfo_callback,
            10,
        )

        self.create_subscription(
            String,
            "/ired_refbox/navigationroutes",
            self.navigationroutes_callback,
            10,
        )

        self.create_subscription(
            String,
            "/ired_refbox/gamestate",
            self.gamestate_callback,
            10,
        )

        self.machineinfo_pub = self.create_publisher(
            String,
            "/ired_refbox/parsed/machineinfo",
            10,
        )

        self.navigationroutes_pub = self.create_publisher(
            String,
            "/ired_refbox/parsed/navigationroutes",
            10,
        )

        self.gamestate_pub = self.create_publisher(
            String,
            "/ired_refbox/parsed/gamestate",
            10,
        )

        self.team_color_pub = self.create_publisher(
            String,
            "/ired_refbox/parsed/team_color",
            10,
        )

        self.challenge_pub = self.create_publisher(
            String,
            "/ired_refbox/challenge_type",
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/ired/refbox_markers",
            10,
        )

        self.create_timer(0.5, self.publish_data)

    def machineinfo_callback(self, msg):
        self.machineinfo = msg.data

    def navigationroutes_callback(self, msg):
        self.navigationroutes = msg.data

    def gamestate_callback(self, msg):
        self.gamestate = msg.data

    def publish_json(self, publisher, data):
        msg = String()
        msg.data = json.dumps(data)
        publisher.publish(msg)

    def parse_machines(self):
        machines = []

        for block in parse_blocks(self.machineinfo, "machines"):
            machine = parse_fields(block)
            machines.append(machine)

        return machines

    def parse_routes(self):
        routes = []

        for block in parse_blocks(self.navigationroutes, "routes"):
            route = parse_fields(block)

            # These fields can occur more than once.
            route["route"] = parse_repeated_values(block, "route")
            route["reached"] = parse_repeated_values(block, "reached")
            route["remaining"] = parse_repeated_values(block, "remaining")

            routes.append(route)

        return {
            "routes": routes,
            "team_color": parse_fields(self.navigationroutes).get("team_color"),
        }

    def parse_game(self, machines):
        game = parse_fields(self.gamestate)

        time_blocks = parse_blocks(self.gamestate, "game_time")

        if time_blocks:
            game["game_time"] = parse_fields(time_blocks[0])

        phase = game.get("phase", "UNKNOWN")

        # The machine information identifies the challenge type:
        # navigation machines have zone/rotation fields;
        # exploration machines do not.
        challenge_type = "UNKNOWN"

        if phase == "PRODUCTION" and machines:
            if any("zone" in machine or "rotation" in machine for machine in machines):
                challenge_type = "NAVIGATION"
            else:
                challenge_type = "EXPLORATION"

        game["challenge_type"] = challenge_type

        return game

    def make_marker(
        self,
        marker_id,
        namespace,
        marker_type,
        x,
        y,
        z,
        scale,
        color,
        text="",
    ):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        marker.text = text
        marker.lifetime.sec = 0

        return marker

    def make_markers(self, machines, routes):
        output = MarkerArray()

        # Remove the previous frame's markers before rebuilding them.
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        output.markers.append(delete_all)

        for index, machine in enumerate(machines):
            name = machine.get("name", f"machine_{index}")
            zone = machine.get("zone")

            if not zone:
                continue

            position = parse_zone(zone)

            if position is None:
                continue

            x, y = position
            is_magenta = name.startswith("M-") or name.startswith("M_")

            if is_magenta:
                color = (1.0, 0.2, 0.65, 0.9)
            else:
                color = (0.0, 0.8, 1.0, 0.9)

            output.markers.append(
                self.make_marker(
                    index,
                    "machines",
                    Marker.CUBE,
                    x,
                    y,
                    0.2,
                    (0.55, 0.55, 0.15),
                    color,
                )
            )

            output.markers.append(
                self.make_marker(
                    index,
                    "machine_labels",
                    Marker.TEXT_VIEW_FACING,
                    x,
                    y,
                    0.65,
                    (0.0, 0.0, 0.3),
                    (1.0, 1.0, 1.0, 1.0),
                    name,
                )
            )

            if "rotation" in machine:
                yaw = math.radians(float(machine["rotation"]))

                arrow = self.make_marker(
                    index,
                    "machine_heading",
                    Marker.ARROW,
                    x,
                    y,
                    0.3,
                    (0.4, 0.08, 0.08),
                    (1.0, 1.0, 0.0, 1.0),
                )

                arrow.pose.orientation.z = math.sin(yaw / 2.0)
                arrow.pose.orientation.w = math.cos(yaw / 2.0)

                output.markers.append(arrow)

        marker_id = 0

        for route in routes["routes"]:
            for zone in route.get("reached", []):
                position = parse_zone(zone)

                if position is None:
                    continue

                x, y = position

                output.markers.append(
                    self.make_marker(
                        marker_id,
                        "reached_zones",
                        Marker.SPHERE,
                        x,
                        y,
                        0.08,
                        (0.22, 0.22, 0.08),
                        (0.1, 0.35, 1.0, 0.9),
                    )
                )

                output.markers.append(
                    self.make_marker(
                        marker_id,
                        "reached_labels",
                        Marker.TEXT_VIEW_FACING,
                        x,
                        y,
                        0.35,
                        (0.0, 0.0, 0.18),
                        (0.1, 0.35, 1.0, 1.0),
                        zone,
                    )
                )

                marker_id += 1

            for zone in route.get("remaining", []):
                position = parse_zone(zone)

                if position is None:
                    continue

                x, y = position

                output.markers.append(
                    self.make_marker(
                        marker_id,
                        "remaining_zones",
                        Marker.SPHERE,
                        x,
                        y,
                        0.08,
                        (0.2, 0.2, 0.08),
                        (1.0, 0.65, 0.0, 0.9),
                    )
                )

                output.markers.append(
                    self.make_marker(
                        marker_id,
                        "remaining_labels",
                        Marker.TEXT_VIEW_FACING,
                        x,
                        y,
                        0.35,
                        (0.0, 0.0, 0.18),
                        (1.0, 0.65, 0.0, 1.0),
                        zone,
                    )
                )

                marker_id += 1

        return output

    def publish_data(self):
        machines = self.parse_machines()
        routes = self.parse_routes()
        game = self.parse_game(machines)

        if self.machineinfo:
            self.publish_json(
                self.machineinfo_pub,
                {"machines": machines},
            )

        if self.navigationroutes:
            self.publish_json(
                self.navigationroutes_pub,
                routes,
            )

        if self.gamestate:
            self.publish_json(
                self.gamestate_pub,
                game,
            )

            challenge_msg = String()
            challenge_msg.data = game["challenge_type"]
            self.challenge_pub.publish(challenge_msg)

        msg = String()
        msg.data = self.team_color
        self.team_color_pub.publish(msg)

        if self.machineinfo or self.navigationroutes:
            self.marker_pub.publish(self.make_markers(machines, routes))


def main(args=None):
    rclpy.init(args=args)
    node = RefboxParser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
