from custom_interfaces.msg import Depthm, Pca9685

import rclpy
import numpy as np
import math
from rclpy.node import Node


def clamp(val: float, min_val: float, max_val: float):
    return max(min_val, min(max_val, val))


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._accum = 0
        self._prev_error = 0
        self._prev_diff = 0
        self._pos_tolerance = 0.05
        self._vel_tolerance = 0.02
        self._minmax_accum = (-200, 200)

    def calculate(self, proc_var: float, setpoint: float, dt: float):
        error = setpoint - proc_var
        val_p = self.kp * error

        # if error changes signs, reset the accumulator
        if math.copysign(1, self._prev_error) != math.copysign(1, error):
            self._accum = 0
        self._accum += error * dt
        self._accum = clamp(self._accum, self._minmax_accum[0], self._minmax_accum[1])
        val_i = self.ki * self._accum

        diff = (error - self._prev_error) / dt
        val_d = self.kd * diff
        self._prev_diff = diff

        self._prev_error = error

        return val_p + val_i + val_d

    def reset(self):
        self._accum = 0
        self._prev_error = 0
        self._prev_diff = 0

    def at_setpoint(self):
        return (
            abs(self._prev_error) <= self._pos_tolerance and
            abs(self._prev_diff) <= self._vel_tolerance
        )


class DepthPIDControl(Node):
    def __init__(self):
        super().__init__("depth_pid_control")
        self.depth_sp_subscription = self.create_subscription(
            Depthm, "depth_setpoint", self.depth_sp_callback, 10
        )
        self.depth_actual_subscription = self.create_subscription(
            Depthm, "bar02/depth", self.depth_actual_callback, 10
        )
        self.depth_actual_subscription
        self.depth_sp_subscription
        self.depth_signal_publisher = self.create_publisher(Pca9685, "depth_signal", 10)
        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period, self.depth_signal_pub_callback
        )
        self.depth_setpoint = 0
        self.depth_actual = 0
        self.pid_controller = PIDController(0.1, 0.1, 0.1)
        print("Depth PID Control Ready!")

    def depth_sp_callback(self, msg):
        self.depth_setpoint = msg.depth_m

    def depth_actual_callback(self, msg):
        self.depth_actual = msg.depth_m

    def publish_depth_signal(self, input_value):
        msg = Pca9685()
        msg.channel_values = np.zeros(16, dtype=np.float32)
        msg.channel_values[4] = input_value
        msg.channel_values[5] = input_value
        self.depth_signal_publisher.publish(msg)

    def depth_signal_pub_callback(self):
        pid_output = self.pid_controller.calculate(
            self.depth_actual, self.depth_setpoint, self.timer_period
        )
        self.get_logger().info(f"At setpoint? {'YES' if self.pid_controller.at_setpoint() else ' NO'}; PID_output = {pid_output:g}")
        pid_scale_factor = 1 / 50
        output_signal = clamp(pid_output * pid_scale_factor, -1, 1)  # from 1 (max upward push) to -1 (max downward push)
        self.publish_depth_signal(output_signal)


def main(args=None):
    rclpy.init(args=args)

    depth_pid_control = DepthPIDControl()

    try:
        rclpy.spin(depth_pid_control)
    except KeyboardInterrupt:
        print("Ctrl-C caught! Exiting...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_pid_control.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
