#     __________  _____ ____     __  __      _ __        
#    / ____/ __ \/ ___// __ \   / / / /___  (_) /_  ____ 
#   / /   / /_/ /\__ \/ /_/ /  / / / / __ \/ / __ \/ __ \
#  / /___/ ____/___/ / ____/  / /_/ / / / / / /_/ / /_/ /
#  \____/_/    /____/_/       \____/_/ /_/_/_.___/\____/ 
                                                       
                                                               
#  Authors: Filippo Samorì, Filippo Ugolini and Daniele Crivellari
#  20/06/2025
#  University of Bologna, Italy
#  License: GPL-3.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from custom_msgs.msg import ImageAndInt

from cv_bridge import CvBridge

import socket
import struct
import bitstruct
import queue
import cv2
import numpy as np
from threading import Thread , Lock
from rclpy.callback_groups import ReentrantCallbackGroup
import math

## TOPICS
TOPIC_IMG = '/cam0/image_raw' 
ODOMETRY_TOPIC = '/orb_slam3/odom'

## Debug
#FPS_FILE = 'fps.txt'

# Custom CPX command
WIFI_POSITION_SENDED = 0x40  # Must match the value in the C-code upload to the Crazyflie

class CrazyflieIMGNode(Node):

    def __init__(self):
        super().__init__('crazyflie_img_node')

        # Declare parameters
        self.declare_parameter("ip", "192.168.4.1")
        self.declare_parameter("port", 5000)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        
        # Count variables
        self.img_count = 0
        self.pose_count = 0
        # Time variable
        self.time = 0

        # Callback group for reentrant callbacks - allows multiple callbacks to be executed concurrently
        self.callback_group = ReentrantCallbackGroup()
        # Mutex for thread safety - It is used to ensure that the socket communication is accessed once at a time
        self.mutex = Lock()

        # Publisher for the image
        self.publisher_ = self.create_publisher(ImageAndInt, TOPIC_IMG, 50)
        # Subscriber to the Odometry topic
        self.subscriber = self.create_subscription(Odometry, ODOMETRY_TOPIC, self.odom_callback, 50, callback_group=self.callback_group)

        ## Routine for reading images
        self.routine = self.create_timer(0.01, self.image_routine, callback_group=self.callback_group)

        self.bridge = CvBridge() # Convert OpenCV images to ROS Image messages

        # Create a TCP socket and connect to the Crazyflie deck
        deck_ip, deck_port = ip , port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        self.get_logger().info(f"Connected to {deck_ip}:{deck_port}")

        # Start a multi-threaded executor to handle callbacks
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        ## Debug
        # with open(FPS_FILE, 'w') as f:
        #     f.write('')

    def rx_bytes(self, size):
        """
        Receive a specific number of bytes from the socket.
            param:
                - size: Number of bytes to receive
        return: Received data as a bytearray
        """

        data = bytearray()
        while len(data) < size:
            packet = self.client_socket.recv(size - len(data))
            if not packet:
                raise RuntimeError("Connection lost")
            data.extend(packet)
        return data
    

    def build_cpx_packet(self, rsv, version, function, source, destination, last_packet, data):
        """
        Build a CPX packet to send to the Crazyflie.
            param:
                - rsv: Reserved field (1 bit)
                - version: Version of the protocol (2 bit)
                - function: Function code (6 bits)
                - source: Source of the packet (3 bits)
                - destination: Destination of the packet (3 bits)
                - last_packet: Last packet flag (1 bit, 0 or 1)
                - data: Payload data as a bytearray
            return: Full CPX packet as a bytearray
        """
        length = 2 + len(data)  # 2 bytes for length + data length
        length_bytes = struct.pack('<H', length)

        fmt = 'u1u1u3u3u2u6'
        # header fields
        header = bitstruct.pack(fmt, rsv, last_packet, source, destination, version, function)

        return length_bytes + header + data

    def send_position(self, client_socket, x, y, z, roll, pitch, yaw, timestamp):
        """
        Send the position data to the Crazyflie.
            param:
                - client_socket: The socket to send the data through
                - x, y, z: Position coordinates
                - roll, pitch, yaw: Orientation angles
                - timestamp: Timestamp of the data
        """
        # Pack the position data into a byte array
        payload = struct.pack('>B6fI', WIFI_POSITION_SENDED, x, y, z, roll, pitch, yaw, timestamp)
        
        # Build the CPX packet
        full_packet = self.build_cpx_packet(rsv=0,
                                             version=0,
                                             function=5,  # CPX_F_APP
                                             source=3,    # CPX_T_WIFI_HOST
                                             destination=4,  # CPX_T_GAP8
                                             last_packet=1,    # bool → 1 byte
                                             data=payload)

        # Send the packet
        client_socket.sendall(full_packet)

    def odom_callback(self, msg):
        """
        Callback function for the Odometry topic.
            param:
                - msg: The Odometry message received
        """
        # Count the number of poses received
        if self.pose_count == 0:
            # Initialize the image count when the first pose is received
            self.img_count = 1
        self.pose_count += 1

        # Extract position and orientation from the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = float(position.x)
        y = float(position.y)
        z = float(position.z)
        # Convert orientation from quaternion to Euler angles (roll, pitch, yaw)

        # Convert quaternion to roll, pitch, yaw
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        timestamp = int(msg.header.stamp.sec)

        # Send the position data to the Crazyflie
        with self.mutex:
            self.send_position(self.client_socket, x, y, z, roll, pitch, yaw, timestamp)

        
    def image_routine(self):
        """
        Routine to read images from the Crazyflie.
        This function reads the image data from the Crazyflie, decodes it, and publishes it as a ROS message.
        """

        try: # Read the image data from the Crazyflie
            # Read the packet header
            with self.mutex:
                packet_info = self.rx_bytes(4)
                length, routing, function = struct.unpack('<HBB', packet_info)
                header = self.rx_bytes(length - 2)
                magic, width, height, depth, fmt, size = struct.unpack('<BHHBBI', header)
                if magic != 0xBC:
                    return 0

                img_stream = bytearray()

                # Read the image data from the stream
                while len(img_stream) < size:
                    chunk_info = self.rx_bytes(4)
                    chunk_len, dst, src = struct.unpack('<HBB', chunk_info)
                    chunk = self.rx_bytes(chunk_len - 2)
                    img_stream.extend(chunk)
            
                # Read the time from the stream
                packetInfoRaw = self.rx_bytes(4)
                [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
                timestamp_data = self.rx_bytes(length - 2)
            # Decode the timestamp
            timestamp = np.frombuffer(timestamp_data, dtype=np.uint32)[0]  

            # Publish the image  
            self.img_publish(img_stream, int(timestamp), fmt)

            ## Compute information for the log
            second , nsecond  = self.get_clock().now().seconds_nanoseconds()
            n_time = second + nsecond*1e-9
            fps = int(1 / (n_time - self.time) if n_time - self.time != 0 else 0)
            self.get_logger().info('Image counter: %d timestamp: %d fps: %d Received Pose Counter: %d' % (self.img_count, timestamp, fps, self.pose_count))
            self.img_count += 1
            # # Debug FPS
            # with open(FPS_FILE, 'a') as f:
            #     f.write(f"{fps}\n")

            self.time = n_time
        except Exception as e:
            self.get_logger().error(f"[Receiver Error] {e}")
            return 0

    def img_publish(self, img_stream, timestamp, fmt):
        """
        Publish the image data as a ROS message.
            param:
                - img_stream: The raw image data received from the Crazyflie
                - timestamp: The timestamp of the image
                - fmt: The format of the image (0 for raw, 1 for JPEG)
        """
        # Custom message type to hold the image and timestamp
        msg = ImageAndInt() # Custom message type
        img = Image()

        if fmt == 0:
            raw = np.frombuffer(img_stream, dtype=np.uint8).reshape((244, 324))
        else:
            arr = np.frombuffer(img_stream, np.uint8)
            raw = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)

        #img = cv2.rotate(raw, cv2.ROTATE_180)
        img = self.bridge.cv2_to_imgmsg(raw, encoding="mono8")
        img.header = Header()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = "cam0"
        
        msg.image = img
        msg.integer = int(timestamp)

        # Publish the image
        self.publisher_.publish(msg)


def main():

    rclpy.init()
    node = CrazyflieIMGNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
