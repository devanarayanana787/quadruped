import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Create a publisher for the f11_joint (one of the "knee" joints)
        self.publisher_ = self.create_publisher(Float64, '/f11_joint/cmd_pos', 10)

        # Create a timer that fires the callback every 0.05 seconds
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Simple Controller started. Publishing to /f11_joint/cmd_pos")
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        msg = Float64()

        # Calculate a sine wave position based on time
        # This will make the joint swing between -0.8 and +0.8 radians
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        amplitude = 0.8  # radians
        frequency = 0.5  # Hz
        msg.data = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
