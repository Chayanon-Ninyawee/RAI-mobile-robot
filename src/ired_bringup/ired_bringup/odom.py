#!/usr/bin/env python3
from math import cos, isnan, sin

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from ired_msgs.msg import IMUData, MotorData
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler


class ODOM(Node):
    def __init__(self):
        super().__init__("odom_node")

        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()

        # Initialise ROS Publisher and Subscriber
        self.odom_data_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.joint_states_pub = self.create_publisher(
            JointState, self.joint_state_topic, 10
        )

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        self.create_subscription(
            MotorData, self.motor_data_topic, self.fetchMotorDataCallback, 10
        )

        self.create_subscription(IMUData, self.rpy_topic, self.fetchIMUCallback, 10)

        self.create_subscription(
            TwistStamped, self.robot_speed_topic, self.fetchRobotSpeedCallback, 10
        )

        # ROS Logger
        self.get_logger().info(
            f"ODOM : ROS publisher on {self.odom_topic} [nav_msgs/Odometry]"
        )
        self.get_logger().info(
            f"ODOM : ROS publisher on {self.joint_state_topic} [sensor_msgs/JointState]"
        )
        self.get_logger().info(
            f"ODOM : ROS subscriber on {self.motor_data_topic} [ired_msgs/MotorData]"
        )
        self.get_logger().info(
            f"ODOM : ROS subscriber on {self.rpy_topic} [ired_msgs/IMUData]"
        )
        self.get_logger().info(
            f"ODOM : ROS subscriber on {self.robot_speed_topic} [geometry_msgs/TwistStamped]"
        )

    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("motor_data_topic", "/ired/motor/data")
        self.declare_parameter("rpy_topic", "/ired/imu/rollpitchyaw")
        self.declare_parameter("robot_speed_topic", "/ired/speed")
        self.declare_parameter("odom_tf_publish", True)
        self.declare_parameter("rpm_to_mps", 0.00664970445)
        self.declare_parameter("wheel_radius", 0.0635)

        # ROS Parameters
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )

        self.joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )

        self.motor_data_topic = (
            self.get_parameter("motor_data_topic").get_parameter_value().string_value
        )

        self.rpy_topic = (
            self.get_parameter("rpy_topic").get_parameter_value().string_value
        )

        self.robot_speed_topic = (
            self.get_parameter("robot_speed_topic").get_parameter_value().string_value
        )

        self.odom_tf_publish = (
            self.get_parameter("odom_tf_publish").get_parameter_value().bool_value
        )

        self.rpm_to_mps = (
            self.get_parameter("rpm_to_mps").get_parameter_value().double_value
        )

        self.wheel_radius = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )

    def initial_variables(self):
        pcov = [
            0.1,
            0,
            0,
            0,
            0,
            0,
            0,
            0.1,
            0,
            0,
            0,
            0,
            0,
            0,
            1e6,
            0,
            0,
            0,
            0,
            0,
            0,
            1e6,
            0,
            0,
            0,
            0,
            0,
            0,
            1e6,
            0,
            0,
            0,
            0,
            0,
            0,
            0.2,
        ]

        self.odom_data = Odometry()
        self.odom_data.header.frame_id = "odom"
        self.odom_data.child_frame_id = "base_footprint"
        self.odom_data.pose.covariance = pcov
        self.odom_data.twist.covariance = pcov

        self.joint_states = JointState()
        self.joint_states.header.frame_id = "base_footprint"
        self.joint_states.name = [
            "wheel_front_left_joint",
            "wheel_front_right_joint",
            "wheel_rear_left_joint",
            "wheel_rear_right_joint",
        ]
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_states.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_states.effort = [0.0, 0.0]

        # Timestamp of the previous /ired/speed sample
        self.prev_speed_time = None

        # Latest sensor data
        self.wheel_speed_cmd = [0.0, 0.0, 0.0, 0.0]
        self.rpy_data = IMUData()

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # Odometry pose
        self.odom_pose = [0.0, 0.0, 0.0]

        # Wheel joint state
        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.last_velocity = [0.0, 0.0, 0.0, 0.0]

        self.X, self.Y, self.Z = 0, 1, 2
        self.FLEFT, self.FRIGHT, self.RLEFT, self.RRIGHT = 0, 1, 2, 3

    def fetchMotorDataCallback(self, msg):
        self.wheel_speed_cmd = list(msg.speed_fb)

    def fetchIMUCallback(self, msg):
        self.rpy_data = msg

    def fetchRobotSpeedCallback(self, msg):
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # First sample only establishes the starting timestamp.
        if self.prev_speed_time is None:
            self.prev_speed_time = current_time

            self.linear_x = msg.twist.linear.x
            self.linear_y = msg.twist.linear.y
            self.angular_z = msg.twist.angular.z

            return

        diff_time = current_time - self.prev_speed_time

        # Ignore invalid timestamps.
        if diff_time.nanoseconds <= 0:
            self.get_logger().warn("Received /ired/speed with invalid timestamp")
            return

        self.prev_speed_time = current_time

        # Store latest velocity.
        self.linear_x = msg.twist.linear.x
        self.linear_y = msg.twist.linear.y
        self.angular_z = msg.twist.angular.z

        # Update odometry.
        self.updateOdometry(diff_time)

        # Use the /ired/speed timestamp everywhere.
        self.odom_data.header.stamp = msg.header.stamp
        self.odom_data_pub.publish(self.odom_data)

        # Update and publish wheel joint state.
        self.updateJoint()

        self.joint_states.header.stamp = msg.header.stamp
        self.joint_states_pub.publish(self.joint_states)

        # Publish odom -> base_footprint TF immediately.
        if self.odom_tf_publish:
            self.publishOdomTF(msg.header.stamp)

    def publishOdomTF(self, stamp):
        qx, qy, qz, qw = quaternion_from_euler(
            0.0,
            0.0,
            self.odom_pose[self.Z],
        )

        odom_tf = TransformStamped()

        odom_tf.header.stamp = stamp
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_footprint"

        odom_tf.transform.translation.x = self.odom_pose[self.X]
        odom_tf.transform.translation.y = self.odom_pose[self.Y]
        odom_tf.transform.translation.z = 0.0

        odom_tf.transform.rotation.x = qx
        odom_tf.transform.rotation.y = qy
        odom_tf.transform.rotation.z = qz
        odom_tf.transform.rotation.w = qw

        odom_tf_msg = TFMessage()
        odom_tf_msg.transforms.append(odom_tf)

        self.tf_pub.publish(odom_tf_msg)

    def updateOdometry(self, diff_time):
        step_time = diff_time.nanoseconds / 1e9

        theta = self.rpy_data.yaw

        # Robot velocity -> world frame
        delta_x = (self.linear_x * cos(theta) - self.linear_y * sin(theta)) * step_time

        delta_y = (self.linear_x * sin(theta) + self.linear_y * cos(theta)) * step_time

        # Update odometry position
        self.odom_pose[self.X] += delta_x
        self.odom_pose[self.Y] += delta_y
        self.odom_pose[self.Z] = theta

        # Update odometry position
        self.odom_data.pose.pose.position.x = self.odom_pose[self.X]
        self.odom_data.pose.pose.position.y = self.odom_pose[self.Y]
        self.odom_data.pose.pose.position.z = 0.0

        # Update odometry orientation
        qx, qy, qz, qw = quaternion_from_euler(
            0.0,
            0.0,
            self.odom_pose[self.Z],
        )

        self.odom_data.pose.pose.orientation.x = qx
        self.odom_data.pose.pose.orientation.y = qy
        self.odom_data.pose.pose.orientation.z = qz
        self.odom_data.pose.pose.orientation.w = qw

        # Robot velocity
        self.odom_data.twist.twist.linear.x = self.linear_x
        self.odom_data.twist.twist.linear.y = self.linear_y
        self.odom_data.twist.twist.angular.z = self.angular_z

        # Wheel joint positions/velocities
        wheel_sign = [1.0, -1.0, 1.0, -1.0]

        for i in range(4):
            wheel_velocity = self.wheel_speed_cmd[i] * wheel_sign[i] * self.rpm_to_mps

            if isnan(wheel_velocity):
                wheel_velocity = 0.0

            wheel_angular_velocity = wheel_velocity / self.wheel_radius

            self.last_velocity[i] = wheel_angular_velocity
            self.last_position[i] += wheel_angular_velocity * step_time

    def updateJoint(self):
        self.joint_states.position = self.last_position
        self.joint_states.velocity = self.last_velocity


def main(args=None):
    rclpy.init(args=args)

    odom_node = ODOM()

    rclpy.spin(odom_node)

    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
