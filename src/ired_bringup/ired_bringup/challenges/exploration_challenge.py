from rclpy.node import Node


class ExplorationChallenge(Node):
    def __init__(self):
        super().__init__("exploration_challenge")

        self.get_logger().info("Exploration challenge selected; implementation pending")

    def stop(self):
        self.get_logger().info("Exploration challenge stopped")
