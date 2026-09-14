#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped
from ired_msgs.msg import IMUData
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler


class IMU(Node):
    def __init__(self):
        super().__init__("imu_node")

        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()

        # Initialise ROS Publisher and Subscriber
        self.imu_data_pub = self.create_publisher(Imu, self.imu_data_topic, 10)
        self.rpy_pub = self.create_publisher(IMUData, self.rpy_topic, 10)
        self.create_subscription(
            TwistStamped, self.imu_raw_topic, self.fetchIMUCallback, 10
        )

        # ROS Logger
        self.get_logger().info(
            f"IMU : ROS publisher on {self.imu_data_topic} [sensor_msgs/Imu]"
        )
        self.get_logger().info(
            f"IMU : ROS publisher on {self.rpy_topic} [ired_msgs/IMUData]"
        )
        self.get_logger().info(
            f"IMU : ROS subscriber on {self.imu_raw_topic} [geometry_msgs/TwistStamped]"
        )

    def initial_parameters(self):
        self.declare_parameter("imu_raw_topic", "/ired/imu/raw")
        self.declare_parameter("imu_data_topic", "/ired/imu/data")
        self.declare_parameter("rpy_topic", "/ired/imu/rollpitchyaw")
        self.declare_parameter("yaw_scale", 1.0)

        self.imu_raw_topic = (
            self.get_parameter("imu_raw_topic").get_parameter_value().string_value
        )
        self.imu_data_topic = (
            self.get_parameter("imu_data_topic").get_parameter_value().string_value
        )
        self.rpy_topic = (
            self.get_parameter("rpy_topic").get_parameter_value().string_value
        )
        self.yaw_scale = self.get_parameter("yaw_scale").value

    def DEG2RAD(self, degree):
        return degree * 0.01745329252

    def initial_variables(self):
        self.imu_data = Imu()
        self.imu_data.header.frame_id = "imu_link"

        self.rpy_data = IMUData()
        self.rpy_data.header.frame_id = "imu_link"
        self.rpy_data.roll = 0.0
        self.rpy_data.pitch = 0.0
        self.rpy_data.yaw = 0.0

        self.angular_velocity = [0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 0.0]

        self.X, self.Y, self.Z = 0, 1, 2

        # Timestamp of the previous IMU sample
        self.prev_imu_time = None

    def fetchIMUCallback(self, msg):
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # First IMU sample.
        # We need two timestamps before we can calculate dt.
        if self.prev_imu_time is None:
            self.prev_imu_time = current_time

            self.angular_velocity[self.X] = msg.twist.angular.x
            self.angular_velocity[self.Y] = msg.twist.angular.y
            self.angular_velocity[self.Z] = msg.twist.angular.z

            self.linear_acceleration[self.X] = msg.twist.linear.x
            self.linear_acceleration[self.Y] = msg.twist.linear.y
            self.linear_acceleration[self.Z] = msg.twist.linear.z

            self.calculateIMU()

            self.imu_data.header.stamp = msg.header.stamp
            self.rpy_data.header.stamp = msg.header.stamp

            self.rpy_pub.publish(self.rpy_data)
            self.imu_data_pub.publish(self.imu_data)

            return

        # Calculate the actual time between IMU samples
        diff_time = current_time - self.prev_imu_time
        self.prev_imu_time = current_time

        # Ignore invalid timestamps
        if diff_time.nanoseconds <= 0:
            self.get_logger().warn("IMU timestamp went backwards or did not advance")
            return

        # Read new IMU data
        self.angular_velocity[self.X] = msg.twist.angular.x
        self.angular_velocity[self.Y] = msg.twist.angular.y
        self.angular_velocity[self.Z] = msg.twist.angular.z

        self.linear_acceleration[self.X] = msg.twist.linear.x
        self.linear_acceleration[self.Y] = msg.twist.linear.y
        self.linear_acceleration[self.Z] = msg.twist.linear.z

        # Integrate using the actual IMU sample interval
        self.calculateRPY(diff_time)

        # Calculate output IMU message
        self.calculateIMU()

        # Both messages use the exact same sensor timestamp
        self.imu_data.header.stamp = msg.header.stamp
        self.rpy_data.header.stamp = msg.header.stamp

        # Publish
        self.rpy_pub.publish(self.rpy_data)
        self.imu_data_pub.publish(self.imu_data)

    def calculateRPY(self, diff_time):
        dt = diff_time.nanoseconds / 1e9

        self.rpy_data.roll += self.DEG2RAD(self.angular_velocity[self.X] * dt)

        self.rpy_data.pitch += self.DEG2RAD(self.angular_velocity[self.Y] * dt)

        self.rpy_data.yaw += self.DEG2RAD(
            self.angular_velocity[self.Z] * self.yaw_scale * dt
        )

    def calculateIMU(self):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.rpy_data.yaw)

        self.imu_data.orientation.x = qx
        self.imu_data.orientation.y = qy
        self.imu_data.orientation.z = qz
        self.imu_data.orientation.w = qw

        self.imu_data.angular_velocity.x = self.DEG2RAD(self.angular_velocity[self.X])
        self.imu_data.angular_velocity.y = self.DEG2RAD(self.angular_velocity[self.Y])
        self.imu_data.angular_velocity.z = self.DEG2RAD(self.angular_velocity[self.Z])

        self.imu_data.linear_acceleration.x = self.linear_acceleration[self.X]
        self.imu_data.linear_acceleration.y = self.linear_acceleration[self.Y]
        self.imu_data.linear_acceleration.z = self.linear_acceleration[self.Z]


def main(args=None):
    rclpy.init(args=args)

    imu_node = IMU()

    rclpy.spin(imu_node)

    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
