import numpy as np
import rclpy
from rclpy.node import Node

from challenge_2_msgs.msg import SignalParameters
from std_msgs.msg import Float32
class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('default.type', rclpy.Parameter.Type.INTEGER),
                ('default.frequency', rclpy.Parameter.Type.DOUBLE),
                ('default.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('default.offset', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.params_publisher = self.create_publisher(SignalParameters, 'signal_params', 10)
        self.signal_pub_frequency = 1000
        self.time_period = 1/self.signal_pub_frequency
        self.timer = self.create_timer(self.time_period, self.signal_timer_callback)
        self.params_pub_frequency = 10
        self.params_pub_time_period = 1/self.params_pub_frequency
        self.params_timer = self.create_timer(self.params_pub_time_period, self.params_timer_callback)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.t = 0
        # print
        self.get_logger().info('Signal Generator Node has been started')
    
    def signal_timer_callback(self):
        
        signal_type = self.get_parameter('default.type').get_parameter_value().integer_value
        signal_frequency = self.get_parameter('default.frequency').get_parameter_value().double_value
        signal_amplitude = self.get_parameter('default.amplitude').get_parameter_value().double_value
        signal_offset = self.get_parameter('default.offset').get_parameter_value().double_value
        
        signal = 0
        #print(f"Signal Type: {signal_type}, Frequency: {signal_frequency}, Amplitude: {signal_amplitude}, Offset: {signal_offset}")
        if signal_type == 1:
            signal = signal_amplitude * np.sin(2 * np.pi * signal_frequency * self.t) + signal_offset
        elif signal_type == 2:
            signal = signal_amplitude * np.sign(np.sin(2 * np.pi * signal_frequency * self.t)) + signal_offset
        elif signal_type == 3:
            # sawtooth wave
            signal = signal_amplitude * (2 * (signal_frequency * self.t - np.floor(signal_frequency * self.t + 0.5))) + signal_offset
        
        signal_msg = Float32()
        signal_msg.data = signal
        #self.get_logger().info(f"Signal: {signal_msg.data}, Time: {self.t}")
        self.signal_publisher.publish(signal_msg)
        self.t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
    
    def params_timer_callback(self):
        signal_params = SignalParameters()
        signal_params.type = self.get_parameter('default.type').get_parameter_value().integer_value
        signal_params.frequency = self.get_parameter('default.frequency').get_parameter_value().double_value
        signal_params.amplitude = self.get_parameter('default.amplitude').get_parameter_value().double_value
        signal_params.offset = self.get_parameter('default.offset').get_parameter_value().double_value
        
        params_msg = SignalParameters()
        params_msg.type = signal_params.type
        params_msg.frequency = signal_params.frequency
        params_msg.amplitude = signal_params.amplitude
        params_msg.offset = signal_params.offset
        params_msg.time = float(self.t)
        self.params_publisher.publish(params_msg)
    

def main(args=None):
    rclpy.init(args=args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    signal_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()