# Usage:
# - change directory into ros2_ws
# - file structure should be src/ghetto_control/ghetto_control/GhettoSub.py
# - build with `colcon build --packages-select ghetto_control; source install/setup.bash`
# - run with `ros2 run ghetto_control ghetto_control`

from dataclasses import dataclass
from enum import Enum

import rclpy
import numpy as np
import math
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy

# basis vectors
basis_f = np.array([1, 1, -1, -1])
basis_r = np.array([-1, 1, -1, 1])
basis_yaw = np.array([-1, 1, 1, -1]) #right, inverse for left

# PWM constants
FREQ_HZ = 50
NEUTRAL = 1500
RANGE_US = 400
SERVO_CLAW_RANGE_US = 300 # 1000 for FT6335M # 300 for HSR-1425CR

# Power constants
VERTICAL_POWER = 0.75
TURN_POWER = 0.75
PWR_MODE = 0

def us_to_value(us):
    return int(4095 * (us / 1000) / (1000 / FREQ_HZ))

def duty_cycle(power, range=RANGE_US, neutral=NEUTRAL):
    return us_to_value(neutral+range*power)

@dataclass
class Channel:
    channel: PwmChannel
    reversed: bool = False
    range: int = RANGE_US
    neutral: int = NEUTRAL
    
    def set(self, power):
        navigator.set_pwm_channel_value(self.channel, duty_cycle((-1 if self.reversed else 1)*power, self.range, self.neutral))

class Channels(Channel, Enum):
    # NAME = (channel, reversed=False, range=400, neutral=1500)
    THRUSTER_FR = (PwmChannel.Ch1, True)
    THRUSTER_FL = (PwmChannel.Ch2, True)
    THRUSTER_BR = (PwmChannel.Ch3, True)
    THRUSTER_BL = (PwmChannel.Ch4, True)
    THRUSTER_VR = (PwmChannel.Ch5, True)
    THRUSTER_VL = (PwmChannel.Ch6, False)
    SERVO_CLAW  = (PwmChannel.Ch7, False, SERVO_CLAW_RANGE_US)


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
        self.left_x = -msg.axes[1] # flipped b/c normally +1 corresponds to left :vomit:
        self.left_y = msg.axes[0]
        self.left_trigger = (1-msg.axes[2])/2
        self.right_x = -msg.axes[4]
        self.right_y = msg.axes[3]
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

class GhettoSubscriber(Node):
    def __init__(self):
        super().__init__('ghetto_sub')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        navigator.init();
        navigator.set_pwm_freq_hz(FREQ_HZ)
        navigator.set_pwm_enable(True)
        
        self.right_edge_detector = EdgeDetector()
        self.left_edge_detector = EdgeDetector()
        self.mode_edge_detector = EdgeDetector()
        self.PWR_MODE = 0

        self.servo_state = 0 # 1 for right, -1 for left
        
        print('Ready!')

    def norm_input(self, input_array, f_scale, r_scale):
        if np.linalg.norm(input_array) == 0:
            return input_array
    
        scale = math.sqrt(2) * max(min(np.linalg.norm(np.array([f_scale, r_scale])), 1), -1)
        direction = input_array / (np.linalg.norm(input_array)) # unit vector

        return scale * direction
    
    def push_to_thrusters(self, input_array):
        Channels.THRUSTER_FR.set(input_array[0])
        Channels.THRUSTER_FL.set(input_array[1])
        Channels.THRUSTER_BR.set(input_array[2])
        Channels.THRUSTER_BL.set(input_array[3])
    
    def push_to_vertical(self, left_power, right_power):
        # 1 is up
        Channels.THRUSTER_VR.set(right_power * VERTICAL_POWER)
        Channels.THRUSTER_VL.set(left_power  * VERTICAL_POWER)
    
    def listener_callback(self, msg):
        xbox = XboxMsg(msg)
        self.mode_edge_detector.update(xbox.start)
        self.left_edge_detector.update(xbox.x)
        self.right_edge_detector.update(xbox.b)

        if self.mode_edge_detector.rising:
            self.PWR_MODE = (self.PWR_MODE + 1) % 3
            self.get_logger().info(f'Power mode: {self.PWR_MODE}')

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
        Channels.SERVO_CLAW.set(self.servo_state)
        
        f_scale = xbox.left_y
        r_scale = xbox.left_x
        PWR_SCALE = 1 / (2 ** self.PWR_MODE)  # this needs to be continuously updated
        if (not f_scale) and (not r_scale) and (xbox.right_bumper == xbox.left_bumper):
            self.push_to_thrusters([0,0,0,0])
        else:
            if xbox.right_bumper:
                output = TURN_POWER * basis_yaw
                self.push_to_thrusters(output)
                self.get_logger().info(f'yawing right: {output}')
            elif xbox.left_bumper:
                output = -TURN_POWER * basis_yaw
                self.push_to_thrusters(output)
                self.get_logger().info(f'yawing left: {output}')
            else:
                # yawing takes exclusive precedence over translational motion 
                output = f_scale * basis_f + r_scale * basis_r
                output = self.norm_input(output, f_scale, r_scale)
                output = output * PWR_SCALE
                # publish
                self.push_to_thrusters(output)
                self.get_logger().info(f'Input: {f_scale:g} {r_scale:g}\tOutput vector: {output}')

        vertical = np.array([0,0])
        vertical_strs = []
        if xbox.dpad_up:
            self.push_to_vertical(1, 1)
            vertical_strs.append('up')
        elif xbox.dpad_down:
            self.push_to_vertical(-1, -1)
            vertical_strs.append('down')
        else:
            self.push_to_vertical(0,0)

        if xbox.dpad_left:
            #vertical += [-1, 1]
            vertical_strs.append('roll left')
        if xbox.dpad_right:
            #vertical += [ 1,-1]
            vertical_strs.append('roll right')
        if len(vertical_strs):
            self.get_logger().info(f'vertical {" + ".join(vertical_strs)}')
        #self.push_to_vertical()

def main(args=None):
    rclpy.init(args=args)

    ghetto_subscriber = GhettoSubscriber()

    try:
        rclpy.spin(ghetto_subscriber)
    except KeyboardInterrupt:
        print('Ctrl-C caught! Exiting...')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ghetto_subscriber.destroy_node()
    navigator.set_pwm_channel_value(PwmChannel.All, 0)
    navigator.set_pwm_enable(False)
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
