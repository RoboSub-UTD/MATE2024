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
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

# basis vectors
basis_f = np.array([1, 1, -1, -1])
basis_r = np.array([-1, 1, -1, 1])
basis_yaw = np.array([-1, 1, 1, -1])  # right, inverse for left

# Power constants
DEPTH_SETPT_DELTA = 0.01
#VERTICAL_POWER = 0.9
TURN_POWER = 0.75  # never set this to 1
DEFAULT_PWR_MODE = 0


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

'''
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
'''

class DS4Msg:
        def __init__(self, msg):
                """
                Joystick: +1 for up, right
                Trigger: 0 to 1 for press amount
                Button: False for unpressed, True for pressed
                """
                self.left_y = -msg.axes[0]
                self.left_x = msg.axes[1]  # flipped b/c normally +1 corresponds to left :vomit:
                self.left_trigger = (1 - msg.axes[2]) / 2
                self.right_y = msg.axes[4]
                self.right_x = -msg.axes[3]
                self.right_trigger = (1 - msg.axes[5]) / 2
                self.dpad_left = msg.axes[6] == 1.0
                self.dpad_right = msg.axes[6] == -1.0
                self.dpad_up = msg.axes[7] == 1.0
                self.dpad_down = msg.axes[7] == -1.0
                button_map = list(map(bool, msg.buttons))
                self.a = button_map[1] ## O button
                self.b = button_map[0] ## X button
                self.x = button_map[2] ## Triangle button
                self.y = button_map[3] ## Square button
                self.left_bumper = button_map[4]
                self.right_bumper = button_map[5]
                self.share = button_map[8]
                self.options = button_map[9]
                self.left_stick = button_map[11]    # press into the left stick
                self.right_stick = button_map[12]  # press into the right stick


class ControllerSignalPub(Node):
    def __init__(self):
        super().__init__("controller_signal_pub")
        self.translational_publisher = self.create_publisher(Pca9685, "translational_signal", 10)
        self.dh_flag_publisher = self.create_publisher(Bool, "depth_hold_flag", 10)
        self.depth_sp_publisher = self.create_publisher(Depthm, "depth_setpoint", 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.depth_publisher_callback)
        self.depth_setpoint = 0
        self.subscription = self.create_subscription(Joy, "joy", self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.sclaw_left_edge_detector = EdgeDetector()
        self.sclaw_right_edge_detector = EdgeDetector()
        self.solenoid_edge_detector = EdgeDetector()
        self.mode_edge_detector = EdgeDetector()
        self.dh_edge_detector = EdgeDetector()
        
        self.dh_flag = False
        
        self.power_mode = DEFAULT_PWR_MODE
        
        self.servo_state = 0 # 1 for right, -1 for left
        self.spool_state = 0
        self.solenoid_state = 0
        
        #self.declare_parameter('manual_depth_control', False)

        #self.manual_depth_control = False
        
        print("Controller Signal Publisher Ready!")

    def norm_input(self, input_array, f_scale, r_scale):
        if np.linalg.norm(input_array) == 0:
            return input_array

        scale = math.sqrt(2) * max(min(np.linalg.norm(np.array([f_scale, r_scale])), 1), -1)
        direction = input_array / (np.linalg.norm(input_array))  # unit vector

        return scale * direction

    def publish_translational(self, pwm_outputs):
        msg = Pca9685()
        msg.channel_values = np.zeros(16, dtype=np.float32)
        msg.channel_values[0:len(pwm_outputs)] = pwm_outputs

        self.translational_publisher.publish(msg)

    def depth_publisher_callback(self):
        msg = Depthm()
        msg.depth_m = float(self.depth_setpoint)
        self.depth_sp_publisher.publish(msg)
    
    def publish_dh_flag(self, dh_flag):
        msg = Bool()
        if dh_flag:
            msg.data = True
            
        else:
            msg.data = False
            
        self.dh_flag_publisher.publish(msg)
            

    def listener_callback(self, msg):
        ds4 = DS4Msg(msg)
        #self.manual_depth_control = self.get_parameter('manual_depth_control').get_parameter_value()
        self.mode_edge_detector.update(ds4.options)
        self.sclaw_left_edge_detector.update(ds4.x)
        self.sclaw_right_edge_detector.update(ds4.b)
        self.solenoid_edge_detector.update(ds4.a)
        self.dh_edge_detector.update(ds4.share)

        if self.mode_edge_detector.rising:
            self.power_mode = (self.power_mode + 1) % 3
            self.get_logger().info(f"Power mode: {self.power_mode}")
            
        if self.dh_edge_detector.rising:
            if self.dh_flag == False:
                self.dh_flag = True
                print("depth hold: ON")
            else:
                self.dh_flag = False
                print("depth hold: OFF")
                
        f_scale = ds4.left_y
        r_scale = ds4.left_x
        power_scale = 1 / (2 ** self.power_mode)
        if ds4.left_bumper != ds4.right_bumper or f_scale or r_scale:
            if ds4.right_bumper:
                thrusters = TURN_POWER * basis_yaw
                self.get_logger().info(f"yawing right: {thrusters}")
            elif ds4.left_bumper:
                thrusters = -TURN_POWER * basis_yaw
                self.get_logger().info(f"yawing left: {thrusters}")
            else:
                thrusters = f_scale * basis_f + r_scale * basis_r
                thrusters = self.norm_input(thrusters, f_scale, r_scale)
                thrusters *= power_scale
                self.get_logger().info(f"thruster output: {thrusters}")
        else:
            thrusters = np.array([0,0,0,0])

        debug_vertical = np.array([0, 0])
        if ds4.dpad_up:
            self.depth_setpoint -= DEPTH_SETPT_DELTA  # depth setpoint is +ve for down
        if ds4.dpad_down:
            self.depth_setpoint += DEPTH_SETPT_DELTA
        if not self.dh_flag:
            vertical_power = (ds4.right_trigger - ds4.left_trigger)
            if abs(vertical_power) < 0.05:
                vertical_power = 0
            debug_vertical = np.array([1, 1]) * vertical_power
            if vertical_power:
                self.get_logger().info(f"debug vert vector: {debug_vertical}")

        if self.sclaw_right_edge_detector.rising:
            if self.servo_state != 1:
                self.servo_state = 1
            else:
                self.servo_state = 0
            self.get_logger().info("servo toggled: state " + str(self.servo_state))
        elif self.sclaw_left_edge_detector.rising:
            if self.servo_state != -1:
                self.servo_state = -1
            else:
                self.servo_state = 0
            self.get_logger().info("servo toggled: state " + str(self.servo_state))
        
        if self.solenoid_edge_detector.rising:
            self.solenoid_state ^= 1
            self.get_logger().info("claw toggled: state " + str(self.solenoid_state))
        
        self.spool_state = 0 ##to be implemented later
        
        output = np.concatenate((thrusters, debug_vertical, [self.servo_state], [self.spool_state], [self.solenoid_state]))
        assert output.shape == (9,)
        self.publish_translational(output)
        self.publish_dh_flag(self.dh_flag)


def main(args=None):
    rclpy.init(args=args)

    controller_signal_pub = ControllerSignalPub()

    try:
        rclpy.spin(controller_signal_pub)
    except KeyboardInterrupt:
        print("Ctrl-C caught! Exiting...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_signal_pub.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
