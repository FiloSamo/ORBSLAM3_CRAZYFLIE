#     __________  _____ ____     __  __      _ __        
#    / ____/ __ \/ ___// __ \   / / / /___  (_) /_  ____ 
#   / /   / /_/ /\__ \/ /_/ /  / / / / __ \/ / __ \/ __ \
#  / /___/ ____/___/ / ____/  / /_/ / / / / / /_/ / /_/ /
#  \____/_/    /____/_/       \____/_/ /_/_/_.___/\____/ 
                                                       
#  Authors: Filippo Samorì, Filippo Ugolini, and Daniele Crivellari
#  20/06/2025
#  University of Bologna, Italy
#  License: GPL-3.0


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import cflib

import math

TOPIC_IMU = '/imu0'
LOG_FILE = 'console_crazyflie.txt'


class CrazyflieIMUNode(Node):

    def __init__(self):
        super().__init__('crazyflie_imu_node')

        # Declare parameters
        self.declare_parameter("URI", "radio://0/86/2M/E7E7E7E7ED")
        URI = self.get_parameter("URI").get_parameter_value().string_value

        self.declare_parameter("log_active", False)
        self.LOG_ACTIVE = self.get_parameter("log_active").get_parameter_value().bool_value

        self.get_logger().info(f'Using URI: {URI}')

        # Publisher for IMU data
        self.publisher_ = self.create_publisher(Imu, TOPIC_IMU, 10) # Puslishing on the topic

        # Init Crazyflie 
        cflib.crtp.init_drivers()
        self.cf = Crazyflie()

        # Register connection callbacks
        self.cf.connected.add_callback(self._on_connect)
        self.cf.connection_failed.add_callback(self._on_connection_failed)
        self.cf.connection_lost.add_callback(self._on_connection_lost)
        self.cf.disconnected.add_callback(self._on_disconnected)
        self.cf.console.receivedChar.add_callback(self.log_console)

        self.get_logger().info('Connecting to Crazyflie...')
        self.cf.open_link(URI)
        # Open a file and write nothing to it
        if self.LOG_ACTIVE:
            # Clear the log file if it exists, or create a new one
            with open(LOG_FILE, 'w') as f:
                f.write('')

            

    def _on_connect(self, _):
        '''
            Callback when the Crazyflie is connected.
        '''
        self.get_logger().info('Connected to Crazyflie, starting IMU logging...')

        # Create a log configuration for the IMU data
        log_conf = LogConfig(name='IMU', period_in_ms=10)
        log_conf.add_variable('acc.x', 'float')
        log_conf.add_variable('acc.y', 'float')
        log_conf.add_variable('acc.z', 'float')
        log_conf.add_variable('gyro.x', 'float')
        log_conf.add_variable('gyro.y', 'float')
        log_conf.add_variable('gyro.z', 'float')

        self.cf.log.add_config(log_conf)
        if log_conf.valid:
            log_conf.data_received_cb.add_callback(self._log_data_callback)
            log_conf.start()
        else:
            self.get_logger().error('Invalid log configuration.')

    def _log_data_callback(self, timestamp, data, _):
        '''
            Callback for receiving log data. Every time a new log entry is received, a new data is published in the topic.
        '''
        # Create a new Imu message
        imu_msg = Imu()

        # self.get_logger().info('Data received: {} '.format(data))

        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu0"

        # Convert raw values into SI units if needed
        imu_msg.linear_acceleration.x = data['acc.x'] * 9.81  # Convert from g to m/s^2
        imu_msg.linear_acceleration.y = data['acc.y'] * 9.81  # Convert from g to m/s^2
        imu_msg.linear_acceleration.z = data['acc.z'] * 9.81  # Convert from g to m/s^2

        imu_msg.angular_velocity.x = math.radians(data['gyro.x'])
        imu_msg.angular_velocity.y = math.radians(data['gyro.y'])
        imu_msg.angular_velocity.z = math.radians(data['gyro.z'])

        self.publisher_.publish(imu_msg)

    def _on_connection_failed(self, link_uri, msg):
        self.get_logger().error(f'Connection failed: {msg}')

    def _on_connection_lost(self, link_uri, msg):
        self.get_logger().warn(f'Connection lost: {msg}')

    def _on_disconnected(self, link_uri):
        self.get_logger().info('Disconnected.')

        
    def log_console(self, text):
        '''
            Callback for receiving console messages from the Crazyflie.
            It logs the message to the console and appends it to a file.
        '''
        # Log the message to the console
        self.get_logger().info(text)
        # If logging is active, append the message to the log file
        if self.LOG_ACTIVE:
            # Append the message to the log file
            with open(LOG_FILE, 'a') as f:
                f.write(text + '\n')
                f.flush()


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cf.close_link()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()