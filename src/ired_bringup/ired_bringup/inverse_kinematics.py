#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped
from ired_msgs.msg import MotorData, MotorSpeed
from rclpy.node import Node


class INVERSEKINEMATICS(Node):
    def __init__(self):
        super().__init__("inverse_kinematics_node")

        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()

        # Initialise ROS Publisher and Subscriber
        self.speed_control_pub = self.create_publisher(
            MotorSpeed, self.inverse_kinematics_topic, 10
        )
        self.create_subscription(
            TwistStamped, self.cmd_vel_topic, self.fetchCommandVelocityCallback, 10
        )
        self.create_subscription(
            MotorData, self.motor_data_topic, self.fetchMotorSpeedCallback, 10
        )

        # Initialise ROS Timers
        timer_period = 0.0333333333  # 30 Hz
        self.timer = self.create_timer(timer_period, self.update_callback)

        # ROS Logger
        self.get_logger().info(
            f"Inverse : ROS publisher on {self.inverse_kinematics_topic} [ired_msgs/MotorSpeed]"
        )
        self.get_logger().info(
            f"Inverse : ROS subscriber on {self.cmd_vel_topic} [geometry_msgs/TwistStamped]"
        )

    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motor_control_topic", "/ired/motor/speed_control")
        self.declare_parameter("wheel_seperation", 0.215)
        self.declare_parameter("mps_to_rpm", 150.38262339391686)

        self.declare_parameter("motor_data_topic", "/ired/motor/data")
        self.declare_parameter("rpm_to_mps", 0.00664970445)

        # ROS Parameters
        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.inverse_kinematics_topic = (
            self.get_parameter("motor_control_topic").get_parameter_value().string_value
        )
        self.wheel_seperation = (
            self.get_parameter("wheel_seperation").get_parameter_value().double_value
        )
        self.mps_to_rpm = (
            self.get_parameter("mps_to_rpm").get_parameter_value().double_value
        )
        self.last_time_cmd_vel = self.get_clock().now()

        self.motor_data_topic = (
            self.get_parameter("motor_data_topic").get_parameter_value().string_value
        )
        self.rpm_to_mps = (
            self.get_parameter("rpm_to_mps").get_parameter_value().double_value
        )
        self.last_time_speed_fb = self.get_clock().now()

        self.pid = [
            PIDController(kp=0.8, ki=0.0, kd=0.0),
            PIDController(kp=0.8, ki=0.0, kd=0.0),
            PIDController(kp=0.8, ki=0.0, kd=0.0),
            PIDController(kp=0.8, ki=0.0, kd=0.0),
        ]
        self.dt = 0.1
        self.last_update_time = self.get_clock().now()

    def initial_variables(self):
        self.goal_linear_velocity_x = 0.0
        self.goal_linear_velocity_y = 0.0
        self.goal_angular_velocity_z = 0.0
        self.wheel_speed_cmd = [0.0, 0.0, 0.0, 0.0]
        self.motor_speed = MotorSpeed()
        self.motor_speed.speed = [0.0, 0.0, 0.0, 0.0]
        self.wheel_speed_fb = [0.0, 0.0, 0.0, 0.0]
        self.FLEFT = 0
        self.FRIGHT = 1
        self.RLEFT = 2
        self.RRIGHT = 3
        self.last_time_cmd_vel = self.get_clock().now()
        self.last_time_speed_fb = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

    def fetchCommandVelocityCallback(self, msg):
        self.last_time_cmd_vel = self.get_clock().now()

        self.goal_linear_velocity_x = msg.twist.linear.x
        self.goal_linear_velocity_y = msg.twist.linear.y
        self.goal_angular_velocity_z = msg.twist.angular.z

    def fetchMotorSpeedCallback(self, msg):
        self.last_time_speed_fb = self.get_clock().now()
        self.wheel_speed_fb = list(msg.speed_fb)

    def calculateMotorPID(self, target_speed):
        motor_command = [0.0] * 4

        for i in range(4):
            motor_command[i] = self.pid[i].update(
                2.7 * target_speed[i], self.wheel_speed_fb[i], self.dt
            )

        return motor_command

    def calculateInverseKinematics(self):
        vx = self.goal_linear_velocity_x
        vy = self.goal_linear_velocity_y
        wz = self.goal_angular_velocity_z

        k = self.wheel_seperation
        s = 1.41421356237

        # Physical wheel velocities
        v_fl = s * (vx - vy) - k * wz
        v_fr = s * (-vx - vy) - k * wz
        v_rl = s * (vx + vy) - k * wz
        v_rr = s * (-vx + vy) - k * wz

        speed_cmd = [0.0] * 4

        # fl: motor and encoder + mean CCW (GOOD)
        # fr: motor and encoder + mean CW  (fix in software)
        # bl: motor and encoder + mean CCW (GOOD)
        # br: motor and encoder + mean CW  (fix in software)

        speed_cmd[self.FLEFT] = v_fl * self.mps_to_rpm
        speed_cmd[self.FRIGHT] = -v_fr * self.mps_to_rpm
        speed_cmd[self.RLEFT] = v_rl * self.mps_to_rpm
        speed_cmd[self.RRIGHT] = -v_rr * self.mps_to_rpm
        # self.get_logger().info(f"FL: {speed_cmd[self.FLEFT]}, FR: {speed_cmd[self.FRIGHT]}, RL: {speed_cmd[self.RLEFT]}, RR: {speed_cmd[self.RRIGHT]}")

        self.wheel_speed_cmd = speed_cmd

    def update_callback(self):
        time_now = self.get_clock().now()

        self.dt = (time_now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = time_now

        if (time_now - self.last_time_cmd_vel).nanoseconds / 1e9 > 1.0:
            self.goal_linear_velocity_x = 0.0
            self.goal_linear_velocity_y = 0.0
            self.goal_angular_velocity_z = 0.0

        if (time_now - self.last_time_speed_fb).nanoseconds / 1e9 > 1.0:
            self.wheel_speed_fb = [0.0] * 4

        self.calculateInverseKinematics()
        # motor_command = self.calculateMotorPID(self.wheel_speed_cmd)
        # self.motor_speed.speed = [round(x, 4) for x in motor_command]
        self.motor_speed.speed = [round(1.7 * x, 4) for x in self.wheel_speed_cmd]
        self.speed_control_pub.publish(self.motor_speed)


class PIDController:
    def __init__(self, kp, ki, kd, output_min=None, output_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def update(self, target, actual, dt):
        if dt <= 0.0:
            return 0.0

        error = target - actual

        # Integral
        self.integral += error * dt

        # Derivative
        if self.initialized:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
            self.initialized = True

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Output limiting
        if self.output_min is not None:
            output = max(self.output_min, output)

        if self.output_max is not None:
            output = min(self.output_max, output)

        self.previous_error = error

        return output


def main(args=None):
    rclpy.init(args=args)
    inverse_kinematics = INVERSEKINEMATICS()

    rclpy.spin(inverse_kinematics)

    inverse_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
