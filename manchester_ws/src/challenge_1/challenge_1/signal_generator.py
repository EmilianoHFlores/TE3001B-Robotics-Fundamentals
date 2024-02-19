import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher = self.create_publisher(Float32, 'time', 10)
        self.frequency = 10
        self.time_period = 1/self.frequency
        self.timer = self.create_timer(self.time_period, self.timer_callback)
        self.t = 0
        # print
        self.get_logger().info('Signal Generator Node has been started')
    
    def timer_callback(self):
        signal_msg = Float32()
        time_msg = Float32()
        signal_msg.data = np.sin(self.t)
        time_msg.data = float(self.t)
        self.get_logger().info(f"Signal: {signal_msg.data}, Time: {time_msg.data}")
        self.signal_publisher.publish(signal_msg)
        self.time_publisher.publish(time_msg)
        self.t += self.time_period
    

def main(args=None):
    rclpy.init(args=args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    signal_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()