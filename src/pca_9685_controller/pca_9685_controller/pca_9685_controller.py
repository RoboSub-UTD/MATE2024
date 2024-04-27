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
import board
import adafruit_pca9685

from rclpy.node import Node 

from custom_interfaces.msg import Pca9685

# PWM constants
FREQ_HZ = 50
NEUTRAL = 1500
RANGE_US = 400
SERVO_CLAW_RANGE_US = 300 # 1000 for FT6335M # 300 for HSR-1425CR

i2c = board.I2C()
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = FREQ_HZ


def us_to_value(us):
    return int(4095 * (us / 1000) / (1000 / FREQ_HZ))

def duty_cycle(power, range=RANGE_US, neutral=NEUTRAL):
    return us_to_value(neutral+range*power)

@dataclass
class Channel:
    channel: adafruit_pca9685.PWMChannel
    reversed: bool = False
    range: int = RANGE_US
    neutral: int = NEUTRAL
    min_duty = int((neutral - range) * FREQ_HZ / 1000000 * 0xFFFF)
    max_duty = int((neutral + range) * FREQ_HZ / 1000000 * 0xFFFF)
    neut_duty = int(min_duty + max_duty) / 2
    range_duty = int(max_duty - neut_duty)
    
    def set(self, power):
       self.channel.duty_cycle = int(((-1 if self.reversed else 1)*power*self.range_duty + self.neut_duty))

@dataclass
class Channels(Channel, Enum):
    THRUSTER_FR = (pca.channels[0], True)
    THRUSTER_FL = (pca.channels[1], True)
    THRUSTER_BR = (pca.channels[2], True)
    THRUSTER_BL = (pca.channels[3], True)
    THRUSTER_VR = (pca.channels[4], False)
    THRUSTER_VL = (pca.channels[5], False)
    SERVO_CLAW  = (pca.channels[6], False, SERVO_CLAW_RANGE_US)
        

class PCA9685Controller(Node):
    def __init__(self):
        super().__init__('pca_9685_controller')
        self.translational_subscription = self.create_subscription(
            Pca9685,
            'translational_signal',
            self.translational_callback,
            10)
        self.depth_control_subscription = self.create_subscription(
            Pca9685,
            'depth_signal',
            self.depth_control_callback,
            10)
        self.translational_subscription  # prevent unused variable warning
        self.depth_control_subscription  # prevent unused variable warning
        self.manual_depth_control = True
        
        print('PCA9685 Controller Ready!')
    
    def push_translational(self, input_array):
        #self.get_logger().info(f'debug input array: {input_array}')
        Channels.THRUSTER_FR.set(input_array[0])
        Channels.THRUSTER_FL.set(input_array[1])
        Channels.THRUSTER_BR.set(input_array[2])
        Channels.THRUSTER_BL.set(input_array[3])
        if self.manual_depth_control:
                Channels.THRUSTER_VR.set(input_array[4])
                Channels.THRUSTER_VL.set(input_array[5])	
        
    def push_depth_control(self, input_array):
        Channels.THRUSTER_VR.set(input_array[4])
        Channels.THRUSTER_VL.set(input_array[5])
    
    def translational_callback(self, msg):
        self.push_translational(msg.channel_values)
    
    def depth_control_callback(self, msg):
        self.push_deph_control(msg.channel_values)
        

def main(args=None):
    rclpy.init(args=args)

    pca_9685_controller = PCA9685Controller()

    try:
        rclpy.spin(pca_9685_controller)
    except KeyboardInterrupt:
        print('Ctrl-C caught! Exiting...')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    PCA9685Controller.destroy_node()
    pca.deinit()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main
    
    
    

