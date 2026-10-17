import json

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ired_bringup.challenges.exploration_challenge import ExplorationChallenge
from ired_bringup.challenges.navigation_challenge import NavigationChallenge


class ChallengeManager(Node):
    def __init__(self):
        super().__init__("challenge_manager")

        self.game_state = {}
        self.active_challenge = None
        self.active_challenge_type = None
        self.executor = None

        self.create_subscription(
            String,
            "/ired_refbox/parsed/gamestate",
            self.gamestate_callback,
            10,
        )

        self.create_timer(0.5, self.update)

        self.get_logger().info("Challenge manager ready")

    def gamestate_callback(self, msg):
        try:
            self.game_state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.game_state = {}

    def get_requested_challenge(self):
        if self.game_state.get("state") != "RUNNING":
            return None

        if self.game_state.get("phase") != "PRODUCTION":
            return None

        challenge_type = self.game_state.get("challenge_type")

        if challenge_type in ("NAVIGATION", "EXPLORATION"):
            return challenge_type

        return None

    def stop_challenge(self):
        if self.active_challenge is None:
            return

        challenge = self.active_challenge
        challenge_type = self.active_challenge_type

        self.get_logger().info(f"Stopping {challenge_type} challenge")

        # Remove the node from the executor before destroying it.
        if self.executor is not None:
            self.executor.remove_node(challenge)

        challenge.stop()
        challenge.destroy_node()

        self.active_challenge = None
        self.active_challenge_type = None

    def start_challenge(self, challenge_type):
        if challenge_type == "NAVIGATION":
            challenge = NavigationChallenge()

        elif challenge_type == "EXPLORATION":
            challenge = ExplorationChallenge()

        else:
            return

        self.active_challenge = challenge
        self.active_challenge_type = challenge_type

        if self.executor is not None:
            self.executor.add_node(challenge)

        self.get_logger().info(f"Started {challenge_type} challenge")

    def update(self):
        requested = self.get_requested_challenge()

        if requested == self.active_challenge_type:
            return

        self.stop_challenge()

        if requested is None:
            self.get_logger().info("Waiting for RUNNING / PRODUCTION challenge")
            return

        self.start_challenge(requested)


def main(args=None):
    rclpy.init(args=args)

    manager = ChallengeManager()
    executor = SingleThreadedExecutor()
    manager.executor = executor
    executor.add_node(manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop_challenge()
        executor.remove_node(manager)
        manager.destroy_node()
        executor.shutdown()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
