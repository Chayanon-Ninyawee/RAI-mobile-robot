#!/usr/bin/env python3
import json

import rclpy
from geometry_msgs.msg import TwistStamped
from ired_msgs.msg import MotorData, MotorSpeed
from rclpy.node import Node
from std_msgs.msg import Bool, String


class INVERSEKINEMATICS(Node):
    def __init__(self):
        super().__init__("inverse_kinematics_node")

        self.initial_parameters()
        self.initial_variables()

        self.speed_control_pub = self.create_publisher(
            MotorSpeed, self.inverse_kinematics_topic, 10
        )

        self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self.nav_command_callback,
            10,
        )
        self.create_subscription(
            TwistStamped,
            self.teleop_cmd_vel_topic,
            self.teleop_command_callback,
            10,
        )
        self.create_subscription(
            MotorData,
            self.motor_data_topic,
            self.fetchMotorSpeedCallback,
            10,
        )
        self.create_subscription(
            Bool,
            "/ired/localization/ready",
            self.localization_callback,
            10,
        )
        self.create_subscription(
            String,
            "/ired_refbox/parsed/gamestate",
            self.gamestate_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / 30.0, self.update_callback)

        self.get_logger().info(f"Nav2 commands: {self.cmd_vel_topic}")
        self.get_logger().info(f"Teleop commands: {self.teleop_cmd_vel_topic}")
        self.get_logger().info(f"Motor commands: {self.inverse_kinematics_topic}")
        self.get_logger().warning("Motor safety gate enabled. Movement starts LOCKED.")

    def initial_parameters(self):
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("teleop_cmd_vel_topic", "/teleop/cmd_vel")
        self.declare_parameter("motor_control_topic", "/ired/motor/speed_control")
        self.declare_parameter("wheel_seperation", 0.215)
        self.declare_parameter("mps_to_rpm", 150.38262339391686)
        self.declare_parameter("motor_data_topic", "/ired/motor/data")
        self.declare_parameter("rpm_to_mps", 0.00664970445)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.teleop_cmd_vel_topic = self.get_parameter("teleop_cmd_vel_topic").value
        self.inverse_kinematics_topic = self.get_parameter("motor_control_topic").value
        self.wheel_seperation = float(self.get_parameter("wheel_seperation").value)
        self.mps_to_rpm = float(self.get_parameter("mps_to_rpm").value)
        self.motor_data_topic = self.get_parameter("motor_data_topic").value
        self.rpm_to_mps = float(self.get_parameter("rpm_to_mps").value)

    def initial_variables(self):
        self.goal_linear_velocity_x = 0.0
        self.goal_linear_velocity_y = 0.0
        self.goal_angular_velocity_z = 0.0

        self.nav_cmd = None
        self.teleop_cmd = None

        self.last_time_nav_cmd = None
        self.last_time_teleop_cmd = None
        self.last_time_speed_fb = self.get_clock().now()
        self.last_time_localization = None
        self.last_time_gamestate = None
        self.last_update_time = self.get_clock().now()

        self.localization_ready = False
        self.game_allows_movement = False
        self.active_command_source = None
        self.last_gate_state = None

        self.wheel_speed_cmd = [0.0] * 4
        self.motor_speed = MotorSpeed()
        self.motor_speed.speed = [0.0] * 4
        self.wheel_speed_fb = [0.0] * 4

        self.FLEFT = 0
        self.FRIGHT = 1
        self.RLEFT = 2
        self.RRIGHT = 3

        self.dt = 1.0 / 30.0

    def nav_command_callback(self, msg):
        self.nav_cmd = msg
        self.last_time_nav_cmd = self.get_clock().now()

    def teleop_command_callback(self, msg):
        self.teleop_cmd = msg
        self.last_time_teleop_cmd = self.get_clock().now()

    def fetchMotorSpeedCallback(self, msg):
        self.last_time_speed_fb = self.get_clock().now()
        self.wheel_speed_fb = list(msg.speed_fb)

    def localization_callback(self, msg):
        self.last_time_localization = self.get_clock().now()
        self.localization_ready = bool(msg.data)

    def gamestate_callback(self, msg):
        self.last_time_gamestate = self.get_clock().now()

        try:
            game = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.game_allows_movement = False
            return

        self.game_allows_movement = (
            game.get("state") == "RUNNING" and game.get("phase") == "PRODUCTION"
        )

    def command_is_fresh(self, timestamp, now, timeout=1.0):
        return timestamp is not None and (now - timestamp).nanoseconds <= int(
            timeout * 1e9
        )

    def select_command(self, now):
        teleop_fresh = self.command_is_fresh(self.last_time_teleop_cmd, now)

        nav_fresh = self.command_is_fresh(self.last_time_nav_cmd, now)

        gamestate_fresh = self.command_is_fresh(self.last_time_gamestate, now, 1.5)

        # Teleop has priority and does not require an allowed RefBox state.
        if teleop_fresh:
            self.active_command_source = "teleop"
            return self.teleop_cmd.twist

        # Nav2 is permitted only during RUNNING / PRODUCTION.
        if nav_fresh and gamestate_fresh and self.game_allows_movement:
            self.active_command_source = "nav2"
            return self.nav_cmd.twist

        self.active_command_source = None
        return None

    def calculateInverseKinematics(self):
        vx = self.goal_linear_velocity_x
        vy = self.goal_linear_velocity_y
        wz = self.goal_angular_velocity_z

        k = self.wheel_seperation
        s = 1.41421356237

        v_fl = s * (vx - vy) - k * wz
        v_fr = s * (-vx - vy) - k * wz
        v_rl = s * (vx + vy) - k * wz
        v_rr = s * (-vx + vy) - k * wz

        # fl: motor and encoder + mean CCW (GOOD)
        # fr: motor and encoder + mean CW  (fix in software)
        # bl: motor and encoder + mean CCW (GOOD)
        # br: motor and encoder + mean CW  (fix in software)

        self.wheel_speed_cmd[self.FLEFT] = v_fl * self.mps_to_rpm
        self.wheel_speed_cmd[self.FRIGHT] = -v_fr * self.mps_to_rpm
        self.wheel_speed_cmd[self.RLEFT] = v_rl * self.mps_to_rpm
        self.wheel_speed_cmd[self.RRIGHT] = -v_rr * self.mps_to_rpm

    def movement_is_allowed(self, now):
        localization_fresh = self.command_is_fresh(self.last_time_localization, now)

        return self.localization_ready and localization_fresh

    def update_callback(self):
        now = self.get_clock().now()

        self.dt = max(
            (now - self.last_update_time).nanoseconds / 1e9,
            1e-6,
        )
        self.last_update_time = now

        if not self.command_is_fresh(self.last_time_speed_fb, now):
            self.wheel_speed_fb = [0.0] * 4

        localization_allowed = self.movement_is_allowed(now)
        command = self.select_command(now)

        if command is not None:
            if self.active_command_source == "teleop":
                allowed = True
            else:
                allowed = self.movement_is_allowed(now)
        else:
            allowed = False

        if allowed and command is not None:
            self.goal_linear_velocity_x = command.linear.x
            self.goal_linear_velocity_y = command.linear.y
            self.goal_angular_velocity_z = command.angular.z
        else:
            self.goal_linear_velocity_x = 0.0
            self.goal_linear_velocity_y = 0.0
            self.goal_angular_velocity_z = 0.0

        if allowed != self.last_gate_state:
            self.last_gate_state = allowed

            if allowed:
                self.get_logger().warning(
                    f"Motor gate OPEN: source={self.active_command_source}"
                )
            else:
                self.get_logger().warning(
                    "Motor gate CLOSED: localization or command unavailable."
                )

        self.calculateInverseKinematics()

        if not allowed:
            self.motor_speed.speed = [0.0] * 4
        else:
            self.motor_speed.speed = [
                round(1.36 * speed, 4) for speed in self.wheel_speed_cmd
            ]

        self.speed_control_pub.publish(self.motor_speed)


def main(args=None):
    rclpy.init(args=args)
    node = INVERSEKINEMATICS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
