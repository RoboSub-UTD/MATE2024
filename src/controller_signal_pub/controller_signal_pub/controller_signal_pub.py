# Usage:
# - change directory into ros2_ws
# - file structure should be src/ghetto_control/ghetto_control/GhettoSub.py
# - build with `colcon build --packages-select ghetto_control; source install/setup.bash`
# - run with `ros2 run ghetto_control ghetto_control`

from custom_interfaces.msg import Depthm, Pca9685

import rclpy
import numpy as np
import math
from rclpy.node import Node

from std_msgs.msg import String 
from sensor_msgs.msg import Joy

# basis vectors
basis_f = np.array([1, 1, -1, -1])
basis_r = np.array([-1, 1, -1, 1])
basis_yaw = np.array([-1, 1, 1, -1]) #right, inverse for left

TURN_POWER = 0.8 #never set this to 1

class EdgeDetector:
        def __init__(self):
                self.last_value = False
                self.rising = False
                self.falling = False

        def update(self, value):
                self.rising = False
                self.falling = False
                if self.last_value != value:
                    if not self.last_value and value:
                        self.rising = True
                    else:
                        self.falling = True
                self.last_value = value

class XboxMsg:
        def __init__(self, msg):
                """
                Joystick: +1 for up, right
                Trigger: 0 to 1 for press amount
                Button: False for unpressed, True for pressed
                """
                self.left_y = msg.axes[0]
                self.left_x = -msg.axes[1] # flipped b/c normally +1 corresponds to left :vomit:
                self.left_trigger = (1-msg.axes[2])/2
                self.right_y = msg.axes[3]
                self.right_x = -msg.axes[4]
                self.right_trigger = (1-msg.axes[5])/2
                self.dpad_left = msg.axes[6] == 1.0
                self.dpad_right = msg.axes[6] == -1.0
                self.dpad_up = msg.axes[7] == 1.0
                self.dpad_down = msg.axes[7] == -1.0
                button_map = list(map(bool, msg.buttons))
                self.a = button_map[0]
                self.b = button_map[1]
                self.x = button_map[2]
                self.y = button_map[3]
                self.left_bumper = button_map[4]
                self.right_bumper = button_map[5]
                self.back = button_map[6]
                self.start = button_map[7]
                self.left_stick = button_map[9]   # press into the left stick
                self.right_stick = button_map[10] # press into the right stick
        

class ControllerSignalPub(Node):
        def __init__(self):
                super().__init__('controller_signal_pub')
                self.translational_publisher = self.create_publisher(Pca9685, 'translational_signal', 10)
                self.depth_sp_publisher = self.create_publisher(Depthm, 'depth_setpoint', 10)
                timer_period = 0.5
                self.timer = self.create_timer(timer_period, self.depth_publisher_callback)
                self.depth_setpoint = 0
                self.subscription = self.create_subscription(
                    Joy,
                    'joy',
                    self.listener_callback,
                    10)
                self.subscription  # prevent unused variable warning
                self.mode_edge_detector = EdgeDetector()
                self.manual_depth_control = True
                print('Controller Signal Publisher Ready!')

        def norm_input(self, input_array, f_scale, r_scale):
                if np.linalg.norm(input_array) == 0:
                    return input_array

                scale = math.sqrt(2) * max(min(np.linalg.norm(np.array([f_scale, r_scale])), 1), -1)
                direction = input_array / (np.linalg.norm(input_array)) # unit vector

                return scale * direction
    
        def publish_translational(self, input_array, debug_depth_control=[0,0]):
                msg = Pca9685()
                msg.channel_values = np.zeros(16, dtype = np.float32)
                msg.channel_values[0] = input_array[0]
                msg.channel_values[1] = input_array[1]
                msg.channel_values[2] = input_array[2]
                msg.channel_values[3] = input_array[3]
                
                #only for manual depth control debugging
                if self.manual_depth_control:
                        msg.channel_values[4] = debug_depth_control[0]
                        msg.channel_values[5] = debug_depth_control[1]
                self.translational_publisher.publish(msg)
    
        def depth_publisher_callback(self):
                msg = Depthm()
                msg.depth_m = float(self.depth_setpoint)
                self.depth_sp_publisher.publish(msg)
            
        def listener_callback(self, msg):
                xbox = XboxMsg(msg)
                self.PWR_MODE = 0

                self.mode_edge_detector.update(xbox.start)
                '''
                self.left_edge_detector.update(xbox.x)
                self.right_edge_detector.update(xbox.b)
                '''

                if self.mode_edge_detector.rising:
                        self.PWR_MODE = (self.PWR_MODE + 1) % 3
                        self.get_logger().info(f'Power mode: {PWR_MODE}')
                #reimplement spinny claw later.. separate publisher?
                '''
                if self.right_edge_detector.rising:
                if self.servo_state != 1:
                self.servo_state = 1
                else:
                self.servo_state = 0
                elif self.left_edge_detector.rising:
                if self.servo_state != -1:
                self.servo_state = -1
                else:
                self.servo_state = 0
                '''
                f_scale = xbox.left_y
                r_scale = xbox.left_x
                PWR_SCALE = 1 / (2 ** self.PWR_MODE)  # this needs to be continuously updated
                debug_vertical = [0,0]
                if xbox.dpad_up:
                        self.depth_setpoint = self.depth_setpoint + 0.01
                        if self.manual_depth_control:
                                debug_vertical = [0.9, 0.9]
                                self.get_logger().info(f'debug v. vector: {debug_vertical}')
                if xbox.dpad_down:
                        self.depth_setpoint = self.depth_setpoint - 0.01
                        if self.manual_depth_control:
                                debug_vertical = [-0.9, -0.9]
                                self.get_logger().info(f'debug v. vector: {debug_vertical}')
                if (not f_scale) and (not r_scale) and (xbox.right_bumper == xbox.left_bumper) and not xbox.dpad_up and not xbox.dpad_down:
                        self.publish_translational([0,0,0,0])
                else:
                        if xbox.right_bumper:
                                output = TURN_POWER * basis_yaw
                                self.publish_translational(output, debug_vertical)
                                self.get_logger().info(f'yawing right: {output}')
                        elif xbox.left_bumper:
                                output = -TURN_POWER * basis_yaw
                                self.publish_translational(output, debug_vertical)
                                self.get_logger().info(f'yawing left: {output}')
                        else:
                                # yawing takes exclusive precedence over translational motion 
                                output = f_scale * basis_f + r_scale * basis_r
                                output = self.norm_input(output, f_scale, r_scale)
                                # publish
                                self.publish_translational(output, debug_vertical)
                                self.get_logger().info(f'Output vector: {output}')

def main(args=None):
    rclpy.init(args=args)

    controller_signal_pub = ControllerSignalPub()

    try:
        rclpy.spin(controller_signal_pub)
    except KeyboardInterrupt:
        print('Ctrl-C caught! Exiting...')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_signal_pub.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main
    
