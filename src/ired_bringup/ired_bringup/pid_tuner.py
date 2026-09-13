import time
import rclpy
from rclpy.node import Node
from ired_msgs.srv import PIDTuning


class PIDTuner(Node):
    def __init__(self):
        super().__init__('pid_tuner')

        self.client = self.create_client(
            PIDTuning,
            '/ired/pid/tuning'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for /ired/pid/tuning...'
            )

    def set_pid(self, motor, kp, ki, kd):
        request = PIDTuning.Request()

        request.motor = motor
        request.kp = kp
        request.ki = ki
        request.kd = kd

        return self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    node = PIDTuner()

    pid_values = {
        "FL": (7.8, 0.0, 0.0),
        "FR": (8.0, 0.0, 0.0),
        "RL": (8.0, 0.0, 0.0),
        "RR": (7.9, 0.0, 0.0),
    }

    for motor, (kp, ki, kd) in pid_values.items():
        future = node.set_pid(motor, kp, ki, kd)

        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None and future.result().status:
            node.get_logger().info(
                f"{motor}: Kp={kp}, Ki={ki}, Kd={kd}"
            )
        else:
            node.get_logger().error(f"{motor} PID update failed")

        time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
