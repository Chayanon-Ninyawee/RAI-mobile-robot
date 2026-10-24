#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

MAX_LINEAR_SPEED = 0.35
MAX_ANGULAR_SPEED = 0.6

STEP_LINEAR = 0.05
STEP_ANGULAR = 0.10

instructions = """
Control Your Robot
---------------------------
Linear movement:
        W
    A   S   D

Rotation:
    Q       E

Speed:
    I : Increase linear and angular speed
    O : Decrease linear and angular speed

SPACE : Stop
CTRL-C : Quit
"""


def get_key():
    fd = sys.stdin.fileno()

    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        return sys.stdin.read(1) if ready else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        self.publisher = self.create_publisher(TwistStamped, "/teleop/cmd_vel", 10)

        self.linear_speed = 0.20
        self.angular_speed = 0.30

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

    def publish_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = self.target_linear_x
        msg.twist.linear.y = self.target_linear_y
        msg.twist.angular.z = self.target_angular_z

        self.publisher.publish(msg)

    def stop_robot(self):
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.publish_cmd()


def main():
    global settings

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()

    print(instructions)

    try:
        while True:
            key = get_key()

            if key == "w":
                node.target_linear_x = clamp(
                    node.target_linear_x + node.linear_speed,
                    -MAX_LINEAR_SPEED,
                    MAX_LINEAR_SPEED,
                )

            elif key == "s":
                node.target_linear_x = clamp(
                    node.target_linear_x - node.linear_speed,
                    -MAX_LINEAR_SPEED,
                    MAX_LINEAR_SPEED,
                )

            elif key == "a":
                node.target_linear_y = clamp(
                    node.target_linear_y + node.linear_speed,
                    -MAX_LINEAR_SPEED,
                    MAX_LINEAR_SPEED,
                )

            elif key == "d":
                node.target_linear_y = clamp(
                    node.target_linear_y - node.linear_speed,
                    -MAX_LINEAR_SPEED,
                    MAX_LINEAR_SPEED,
                )

            elif key == "q":
                node.target_angular_z = clamp(
                    node.target_angular_z + node.angular_speed,
                    -MAX_ANGULAR_SPEED,
                    MAX_ANGULAR_SPEED,
                )

            elif key == "e":
                node.target_angular_z = clamp(
                    node.target_angular_z - node.angular_speed,
                    -MAX_ANGULAR_SPEED,
                    MAX_ANGULAR_SPEED,
                )

            elif key == "i":
                node.linear_speed = min(
                    node.linear_speed + STEP_LINEAR,
                    MAX_LINEAR_SPEED,
                )
                node.angular_speed = min(
                    node.angular_speed + STEP_ANGULAR,
                    MAX_ANGULAR_SPEED,
                )

            elif key == "o":
                node.linear_speed = max(node.linear_speed - STEP_LINEAR, 0.0)
                node.angular_speed = max(node.angular_speed - STEP_ANGULAR, 0.0)

            elif key in (" ",):
                node.stop_robot()

            elif key == "\x03":
                break

            node.publish_cmd()

            print(
                f"Linear: x={node.target_linear_x:.2f}, "
                f"y={node.target_linear_y:.2f} m/s | "
                f"Angular: z={node.target_angular_z:.2f} rad/s | "
                f"Speed: linear={node.linear_speed:.2f}, "
                f"angular={node.angular_speed:.2f}",
                end="\r",
                flush=True,
            )

    except Exception as e:
        print("\nError:", e)

    finally:
        node.stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nShutting down teleop keyboard.")


if __name__ == "__main__":
    main()
