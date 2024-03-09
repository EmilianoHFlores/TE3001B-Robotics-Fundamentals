import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32

class Process(Node):
    def __init__(self):
        super().__init__('process')
        
        # Obtainable parameters
        self.signal = 0
        self.time = 0
        self.signal_max = {
            "value": 0,
            "time": 0
        }
        self.signal_min = {
            "value": 0,
            "time": 0
        }
        
        # Hardcoded parameters
        self.period = 1 # * pi
        self.shift_vertical = 0
        
        # transforms
        self.amplitude_factor = 0.5
        self.shift_horizontal = 0.75
        
        # formula is amplitude = signal_max - signal_min
        # formula is shift_vertical = (signal_max + signal_min) / 2
        # full formula is amplitude * sin(period*pi + shift_horizontal) + shift_vertical
        
        self.frequency = 10
        self.time_period = 1/self.frequency
        
        self.signal_subscriber = self.create_subscription(Float32, 'signal', self.signal_callback, 10)
        self.time_subscriber = self.create_subscription(Float32, '/time', self.time_callback, 10)
        self.processed_signal_publisher = self.create_publisher(Float32, 'proc_signal', 10)
        self.timer = self.create_timer(self.time_period, self.timer_callback)
        self.get_logger().info('Process Node has been started')

    def signal_callback(self, msg):
        self.signal = msg.data
        if self.signal > self.signal_max["value"]:
            self.signal_max["value"] = self.signal
            self.signal_max["time"] = self.time
        if self.signal < self.signal_min["value"]:
            self.signal_min["value"] = self.signal
            self.signal_min["time"] = self.time
        
    def time_callback(self, msg):
        self.time = msg.data
        
    def timer_callback(self):
        
        # self.period = self.signal_max["time"] - self.signal_min["time"]
        middle = (self.signal_max["value"] + self.signal_min["value"]) / 2
        amplitude = abs(self.signal_max["value"] - self.signal_min["value"]) / 2
        
        new_amplitude = amplitude * self.amplitude_factor
        new_min = middle - new_amplitude
        shift_vertical = -new_min if new_min < 0 else 0
        
        processed_signal_msg = Float32()
        self.get_logger().info(f"Period: {self.period} Amplitude: {amplitude}, Shift Vertical: {shift_vertical}, max: {self.signal_max}, min: {self.signal_min}, time: {self.time}")
        processed_signal_msg.data = new_amplitude * np.sin(self.period * self.time + self.shift_horizontal) + shift_vertical
        self.get_logger().info(f"Processed Signal: {processed_signal_msg.data}")   
        self.processed_signal_publisher.publish(processed_signal_msg)
        
def main(args=None):
    rclpy.init(args=args)
    process = Process()
    rclpy.spin(process)
    process.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()