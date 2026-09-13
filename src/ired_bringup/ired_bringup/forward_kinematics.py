#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped
from ired_msgs.msg import MotorData
from rclpy.node import Node


class FORWARDKINEMATICS(Node):
    def __init__(self):
        super().__init__("forward_kinematics_node")

        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()

        # Initialise ROS Publisher and Subscriber
        self.robot_speed_pub = self.create_publisher(
            TwistStamped, self.forward_kinematics_topic, 10
        )
        self.create_subscription(
            MotorData, self.motor_data_topic, self.fetchMotorSpeedCallback, 10
        )

        # Initialise ROS Timers
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.update_callback)

        # ROS Logger
        self.get_logger().info(
            f"Forward : ROS publisher on {self.forward_kinematics_topic} [geometry_msgs/TwistStamped]"
        )
        self.get_logger().info(
            f"Forward : ROS subscriber on {self.motor_data_topic} [ired_msgs/MotorData]"
        )

    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter("motor_data_topic", "/ired/motor/data")
        self.declare_parameter("robot_speedtopic", "/ired/speed")
        self.declare_parameter("wheel_seperation", 0.215)
        self.declare_parameter("rpm_to_mps", 0.00664970445)

        # ROS Parameters
        self.motor_data_topic = (
            self.get_parameter("motor_data_topic").get_parameter_value().string_value
        )
        self.forward_kinematics_topic = (
            self.get_parameter("robot_speedtopic").get_parameter_value().string_value
        )
        self.wheel_seperation = (
            self.get_parameter("wheel_seperation").get_parameter_value().double_value
        )
        self.rpm_to_mps = (
            self.get_parameter("rpm_to_mps").get_parameter_value().double_value
        )

    def initial_variables(self):
        self.wheel_speed_fb = [0.0, 0.0, 0.0, 0.0]
        self.robot_speed = TwistStamped()
        self.robot_speed.twist.linear.x = 0.0
        self.robot_speed.twist.linear.y = 0.0
        self.robot_speed.twist.linear.z = 0.0
        self.robot_speed.twist.angular.x = 0.0
        self.robot_speed.twist.angular.y = 0.0
        self.robot_speed.twist.angular.z = 0.0
        self.FLEFT = 0
        self.FRIGHT = 1
        self.RLEFT = 2
        self.RRIGHT = 3
        self.last_time_speed_fb = self.get_clock().now()

    def fetchMotorSpeedCallback(self, msg):
        self.last_time_speed_fb = self.get_clock().now()
        self.wheel_speed_fb = list(msg.speed_fb)

    def calculateForwardKinematics(self):
        # fl: motor and encoder + mean CCW (GOOD)
        # fr: motor and encoder + mean CW  (fix in software)
        # bl: motor and encoder + mean CCW (GOOD)
        # br: motor and encoder + mean CW  (fix in software)

        encoder_compensation_mult = 2.0

        v_fl = (
            self.wheel_speed_fb[self.FLEFT]
            * self.rpm_to_mps
            * encoder_compensation_mult
        )
        v_fr = (
            -self.wheel_speed_fb[self.FRIGHT]
            * self.rpm_to_mps
            * encoder_compensation_mult
        )
        v_rl = (
            self.wheel_speed_fb[self.RLEFT]
            * self.rpm_to_mps
            * encoder_compensation_mult
        )
        v_rr = (
            -self.wheel_speed_fb[self.RRIGHT]
            * self.rpm_to_mps
            * encoder_compensation_mult
        )

        k = self.wheel_seperation
        s = 1.41421356237

        vx = (v_fl - v_fr + v_rl - v_rr) / (4.0 * s)
        vy = (-v_fl - v_fr + v_rl + v_rr) / (4.0 * s)
        wz = -(v_fl + v_fr + v_rl + v_rr) / (4.0 * k)

        self.robot_speed.twist.linear.x = vx
        self.robot_speed.twist.linear.y = vy
        self.robot_speed.twist.angular.z = wz

    # def calculateForwardKinematics(self):
    #    v_fl = self.wheel_speed_fb[self.FLEFT]
    #    v_fr = self.wheel_speed_fb[self.FRIGHT]
    #    v_rl = self.wheel_speed_fb[self.RLEFT]
    #    v_rr = self.wheel_speed_fb[self.RRIGHT]

    #    self.robot_speed.twist.linear.x = v_fl
    #    self.robot_speed.twist.linear.y = v_fr
    #    self.robot_speed.twist.linear.z = v_rl
    #    self.robot_speed.twist.angular.z = v_rr

    def update_callback(self):
        time_now = self.get_clock().now()

        if (time_now - self.last_time_speed_fb).nanoseconds / 1e9 > 1.0:
            self.wheel_speed_fb = [0.0] * 4

        self.calculateForwardKinematics()
        self.robot_speed.header.stamp = self.get_clock().now().to_msg()
        self.robot_speed_pub.publish(self.robot_speed)


def main(args=None):
    rclpy.init(args=args)
    forward_kinematics = FORWARDKINEMATICS()

    rclpy.spin(forward_kinematics)

    forward_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
