import os

import rclpy
from rclpy.node import Node


class MapGenerator(Node):
    def __init__(self):
        super().__init__("map_generator")

        self.declare_parameter("width", 5.0)
        self.declare_parameter("height", 5.0)
        self.declare_parameter("resolution", 0.01)
        self.declare_parameter("wall_thickness", 0.03)
        self.declare_parameter("output_dir", "/tmp/ired_map")

        width = float(self.get_parameter("width").value)
        height = float(self.get_parameter("height").value)
        resolution = float(self.get_parameter("resolution").value)
        wall_thickness = float(self.get_parameter("wall_thickness").value)
        output_dir = self.get_parameter("output_dir").value

        if width <= 0 or height <= 0 or resolution <= 0:
            raise ValueError("Map dimensions and resolution must be positive")
        if wall_thickness <= 0:
            raise ValueError("Wall thickness must be positive")

        width_cells = round(width / resolution)
        height_cells = round(height / resolution)
        wall_cells = max(1, round(wall_thickness / resolution))

        # Expand the map so walls extend outside the requested interior.
        map_width_cells = width_cells + 2 * wall_cells
        map_height_cells = height_cells + 2 * wall_cells

        if width_cells < 1 or height_cells < 1:
            raise ValueError("Map must contain at least one cell")

        os.makedirs(output_dir, exist_ok=True)

        pgm_path = os.path.join(output_dir, "map.pgm")
        yaml_path = os.path.join(output_dir, "map.yaml")

        with open(pgm_path, "wb") as image:
            image.write(f"P5\n{map_width_cells} {map_height_cells}\n255\n".encode())

            for row in range(map_height_cells):
                pixels = bytearray([254] * map_width_cells)

                if row < wall_cells or row >= map_height_cells - wall_cells:
                    pixels[:] = bytes([0]) * map_width_cells
                else:
                    pixels[:wall_cells] = bytes([0]) * wall_cells
                    pixels[-wall_cells:] = bytes([0]) * wall_cells

                image.write(pixels)

        # Shift the origin so the interior still begins at (0, 0).
        with open(yaml_path, "w") as config:
            config.write(
                f"image: map.pgm\n"
                f"mode: trinary\n"
                f"resolution: {resolution}\n"
                f"origin: [{-wall_cells * resolution}, "
                f"{-wall_cells * resolution}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )

        self.get_logger().info(
            f"Generated {width} x {height} m interior with "
            f"{wall_thickness} m walls on both sides. "
            f"Total map: {map_width_cells * resolution} x "
            f"{map_height_cells * resolution} m. "
            f"Map: {yaml_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapGenerator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
