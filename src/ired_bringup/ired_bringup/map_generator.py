import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MapGenerator(Node):
    def __init__(self):
        super().__init__("map_generator")

        self.declare_parameter("width", 5.0)
        self.declare_parameter("height", 5.0)
        self.declare_parameter("resolution", 0.01)
        self.declare_parameter("wall_thickness", 0.03)
        self.declare_parameter("output_dir", "/tmp/ired_map")

        self.width = float(self.get_parameter("width").value)
        self.height = float(self.get_parameter("height").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.wall_thickness = float(self.get_parameter("wall_thickness").value)
        self.output_dir = str(self.get_parameter("output_dir").value)

        self.validate_parameters()

        self.generated = False
        self.map_msg = None

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/map",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.create_subscription(
            String,
            "/ired_refbox/parsed/team_color",
            self.team_color_callback,
            10,
        )

        self.create_timer(1.0, self.publish_map)

        self.get_logger().info(
            "Waiting for team color on /ired_refbox/parsed/team_color..."
        )

    def validate_parameters(self):
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0:
            raise ValueError("Map dimensions and resolution must be positive")

        if self.wall_thickness <= 0:
            raise ValueError("Wall thickness must be positive")

        if not self.output_dir:
            raise ValueError("output_dir must not be empty")

    def team_color_callback(self, msg):
        if self.generated:
            return

        team_color = msg.data.strip().upper()

        if team_color not in ("CYAN", "MAGENTA"):
            self.get_logger().warning(f"Ignoring invalid team color: {msg.data!r}")
            return

        self.get_logger().info(f"Received team color: {team_color}")

        try:
            self.generate_map(team_color)
            self.generated = True

            self.get_logger().info(
                "Map generation completed successfully; publishing on /map"
            )

        except (OSError, ValueError) as error:
            self.get_logger().fatal(f"Map generation failed: {error}")
            raise

    def publish_map(self):
        if self.map_msg is None:
            return

        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def generate_map(self, team_color):
        resolution = self.resolution

        width_cells = round(self.width / resolution)
        height_cells = round(self.height / resolution)
        wall_cells = max(1, round(self.wall_thickness / resolution))

        if width_cells < 1 or height_cells < 1:
            raise ValueError("Map must contain at least one cell")

        # Use cell-aligned dimensions so the map metadata matches the PGM.
        field_width = width_cells * resolution
        field_height = height_cells * resolution
        wall_thickness = wall_cells * resolution

        map_width_cells = width_cells + wall_cells
        map_height_cells = height_cells + wall_cells

        # Outer map edges extend half a wall thickness beyond the field.
        origin_x = -wall_thickness / 2.0
        origin_y = -wall_thickness / 2.0

        map_width = map_width_cells * resolution
        map_height = map_height_cells * resolution

        def inside(value, lower, upper):
            return lower <= value < upper

        def in_wall_segment(x, y):
            half_wall = wall_thickness / 2.0

            # Bottom and top perimeter walls.
            if inside(y, -half_wall, half_wall) or inside(
                y,
                field_height - half_wall,
                field_height + half_wall,
            ):
                return True

            # Left perimeter wall.
            if inside(x, -half_wall, half_wall):
                return True

            # Right perimeter wall, with openings beside C_Z51 and C_Z53.
            if inside(
                x,
                field_width - half_wall,
                field_width + half_wall,
            ):
                # C_Z51: y = 0 to 1.
                # C_Z53: y = 2 to 3.
                if not (inside(y, 0.0, 1.0) or inside(y, 2.0, 3.0)):
                    return True

            # Start-zone wall above C_Z51 and C_Z41.
            # Horizontal segment: x = 3 to 5, y = 1.
            if inside(x, 3.0, field_width) and inside(
                y, 1.0 - half_wall, 1.0 + half_wall
            ):
                return True

            # Start-zone wall to the left of C_Z31.
            # Vertical segment: x = 2, y = 0 to 1.
            if inside(x, 2.0 - half_wall, 2.0 + half_wall) and inside(y, 0.0, 1.0):
                return True

            return False

        os.makedirs(self.output_dir, exist_ok=True)

        pgm_path = os.path.join(self.output_dir, "map.pgm")
        yaml_path = os.path.join(self.output_dir, "map.yaml")

        with open(pgm_path, "wb") as image:
            image.write(f"P5\n{map_width_cells} {map_height_cells}\n255\n".encode())

            # PGM rows run from top to bottom, while map y runs bottom to top.
            for row in range(map_height_cells):
                y = origin_y + map_height - (row + 0.5) * resolution

                pixels = bytearray(map_width_cells)

                for column in range(map_width_cells):
                    x = origin_x + (column + 0.5) * resolution

                    # 0 = occupied wall; 254 = free space.
                    pixels[column] = 0 if in_wall_segment(x, y) else 254

                # Mirror the entire map across the Y-axis for Magenta.
                if team_color == "MAGENTA":
                    pixels.reverse()

                image.write(pixels)

        if team_color == "MAGENTA":
            origin_x = -(field_width + wall_thickness / 2.0)

        with open(yaml_path, "w") as config:
            config.write(
                f"image: map.pgm\n"
                f"mode: trinary\n"
                f"resolution: {resolution}\n"
                f"origin: [{origin_x}, {origin_y}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )

        # Build the ROS OccupancyGrid in bottom-to-top row order.
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "map"

        map_msg.info.resolution = resolution
        map_msg.info.width = map_width_cells
        map_msg.info.height = map_height_cells

        map_msg.info.origin.position.x = origin_x
        map_msg.info.origin.position.y = origin_y
        map_msg.info.origin.orientation.w = 1.0

        occupancy_data = []

        for row in range(map_height_cells):
            y = -wall_thickness / 2.0 + (row + 0.5) * resolution
            row_data = []

            for column in range(map_width_cells):
                x = -wall_thickness / 2.0 + (column + 0.5) * resolution

                # 100 = occupied; 0 = free.
                row_data.append(100 if in_wall_segment(x, y) else 0)

            # Mirror the entire map across the Y-axis for Magenta.
            if team_color == "MAGENTA":
                row_data.reverse()

            occupancy_data.extend(row_data)

        map_msg.data = occupancy_data
        self.map_msg = map_msg

        self.get_logger().info(
            f"Generated {team_color} map\n"
            f"  Field: {field_width:.3f} x {field_height:.3f} m\n"
            f"  Wall thickness: {wall_thickness:.3f} m\n"
            f"  Map size: {map_width:.3f} x {map_height:.3f} m\n"
            f"  Origin: ({origin_x:.3f}, {origin_y:.3f})\n"
            f"  Map file: {yaml_path}\n"
            f"  ROS topic: /map (1 Hz)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapGenerator()

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
