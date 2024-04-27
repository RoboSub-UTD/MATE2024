from custom_interfaces.msg import Depthm, Pca9685

import rclpy
import numpy as np
import math
from rclpy.node import Node


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

class DepthPIDControl(Node):
        def __init__(self):
                super().__init__('depth_pid_control')
                self.depth_sp_subscription = self.create_subscription(
                    Depthm,
                    'depth_setpoint',
                    self.depth_sp_callback,
                    10)
                self.depth_actual_subscription = self.create_subscription(
                    Depthm,
                    'bar02/depth',
                    self.depth_actual_callback,
                    10)
                self.depth_actual_subscription
                self.depth_sp_subscription
                self.depth_signal_publisher = self.create_publisher(Pca9685, 'depth_signal', 10)
                self.timer_period = 0.1
                self.timer = self.create_timer(self.timer_period, self.depth_signal_pub_callback)
                self.depth_setpoint = 0;
                self.depth_actual = 0;
                self.PID_Controller = PID(0.1,0.1,0.1)
                print('Depth PID Control Ready!')

        def depth_sp_callback(self, msg):
                self.depth_setpoint = msg.depth_m

        def depth_actual_callback(self, msg):
                self.depth_actual = msg.depth_m

        def publish_depth_signal(self, input_value):
                msg = Pca9685()
                msg.channel_values = np.zeros(16, dtype = np.float32)
                msg.channel_values[4] = input_value
                msg.channel_values[5] = input_value
                self.depth_signal_publisher.publish(msg)
                
        def depth_signal_pub_callback(self):
                output_signal = 0 #from 1 (max upward push) to -1 (max downward push)
                error = self.depth_actual - self.depth_setpoint
                pid_output = self.PID_Controller.update(error, self.timer_period)
                self.get_logger().info('PID_output = ' + str(pid_output))
                pid_scale_factor = 50
                output_signal = pid_output / pid_scale_factor
                output_signal = 1 if output_signal > 1 else output_signal
                output_signal = -1 if output_signal < -1 else output_signal #normalize to max and min outputs..
                self.publish_depth_signal(output_signal)

def main(args=None):
    rclpy.init(args=args)

    depth_pid_control = DepthPIDControl()

    try:
        rclpy.spin(depth_pid_control)
    except KeyboardInterrupt:
        print('Ctrl-C caught! Exiting...')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_pid_control.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main
    
