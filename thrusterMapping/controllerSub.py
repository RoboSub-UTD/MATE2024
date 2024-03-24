import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Joy

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.1, self.publish_joystick)
    
    def publish_joystick(self):
        # Here you would read the joystick inputs from your device
        # For the sake of demonstration, we'll create a fake joystick data
        # Replace this with your actual joystick reading code
        axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg = Joy()
        msg.axes = axes
        msg.buttons = buttons
        self.publisher_.publish(msg)
        self.get_logger().info("Published joystick data")
