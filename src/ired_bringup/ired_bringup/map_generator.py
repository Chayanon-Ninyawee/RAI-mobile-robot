import os

import rclpy
from rclpy.node import Node
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

        self.team_color = None
        self.generated = False

        self.create_subscription(
            String,
            "/ired_refbox/parsed/team_color",
            self.team_color_callback,
            10,
        )

        self.get_logger().info(
            "Waiting for team color on " "/ired_refbox/parsed/team_color..."
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

        self.team_color = team_color

        self.get_logger().info(f"Received team color: {self.team_color}")

        try:
            self.generate_map()
            self.generated = True
            self.get_logger().info("Map generation completed successfully")
            rclpy.shutdown()
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Map generation failed: {error}")
            raise

    def generate_map(self):
        width_cells = round(self.width / self.resolution)
        height_cells = round(self.height / self.resolution)
        wall_cells = max(
            1,
            round(self.wall_thickness / self.resolution),
        )

        if width_cells < 1 or height_cells < 1:
            raise ValueError("Map must contain at least one cell")

        map_width_cells = width_cells + 2 * wall_cells
        map_height_cells = height_cells + 2 * wall_cells

        os.makedirs(self.output_dir, exist_ok=True)

        pgm_path = os.path.join(self.output_dir, "map.pgm")
        yaml_path = os.path.join(self.output_dir, "map.yaml")

        with open(pgm_path, "wb") as image:
            image.write(f"P5\n{map_width_cells} {map_height_cells}\n255\n".encode())

            for row in range(map_height_cells):
                pixels = bytearray([254] * map_width_cells)

                if row < wall_cells or row >= map_height_cells - wall_cells:
                    pixels[:] = bytes(map_width_cells)
                else:
                    pixels[:wall_cells] = bytes(wall_cells)
                    pixels[-wall_cells:] = bytes(wall_cells)

                image.write(pixels)

        # Mirror Cyan across the Y-axis for Magenta.
        map_width = map_width_cells * self.resolution
        map_height = map_height_cells * self.resolution

        if self.team_color == "MAGENTA":
            origin_x = -map_width
        else:
            origin_x = 0.0

        origin_y = 0.0

        with open(yaml_path, "w") as config:
            config.write(
                f"image: map.pgm\n"
                f"mode: trinary\n"
                f"resolution: {self.resolution}\n"
                f"origin: [{origin_x}, {origin_y}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )

        self.get_logger().info(
            f"Generated {self.width} x {self.height} m interior "
            f"for {self.team_color} with "
            f"{self.wall_thickness} m walls. "
            f"Total map: "
            f"{map_width_cells * self.resolution} x "
            f"{map_height_cells * self.resolution} m. "
            f"Map: {yaml_path}"
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
