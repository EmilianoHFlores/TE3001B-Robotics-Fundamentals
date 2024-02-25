import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32
from challenge_2_msgs.msg import SignalParameters

class Process(Node):
    def __init__(self):
        super().__init__('process')
        
        self.signal_type = 0
        self.signal_frequency = 0
        self.signal_amplitude = 0
        self.signal_offset = 0
        self.t = 0
        
        self.reconstructed_signal_publisher = self.create_publisher(Float32, 'reconstructed_signal', 10)
        self.params_subscriber = self.create_subscription(SignalParameters, 'signal_params', self.params_callback, 10)
        
        self.signal_pub_rate = 1000
        self.time_period = 1/self.signal_pub_rate
        self.timer = self.create_timer(self.time_period, self.reconstruct_signal_callback)
        
        
        
    def params_callback(self, msg):
        self.signal_type = msg.type
        self.signal_frequency = msg.frequency
        self.signal_amplitude = msg.amplitude
        self.signal_offset = msg.offset
        self.t = msg.time # reset time to sincronize with the signal generator
    
    def reconstruct_signal_callback(self):
        signal = 0
        if self.signal_type == 1:
            signal = self.signal_amplitude * np.sin(2 * np.pi * self.signal_frequency * self.t) + self.signal_offset
        elif self.signal_type == 2:
            signal = self.signal_amplitude * np.sign(np.sin(2 * np.pi * self.signal_frequency * self.t)) + self.signal_offset
        elif self.signal_type == 3:
            # sawtooth wave
            signal = self.signal_amplitude * (2 * (self.signal_frequency * self.t - np.floor(self.signal_frequency * self.t + 0.5))) + self.signal_offset
        else:
            signal = 0
        signal_msg = Float32()
        signal_msg.data = float(signal)
        self.reconstructed_signal_publisher.publish(signal_msg)
        self.t += self.time_period
        
def main(args=None):
    rclpy.init(args=args)
    process = Process()
    rclpy.spin(process)
    process.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()