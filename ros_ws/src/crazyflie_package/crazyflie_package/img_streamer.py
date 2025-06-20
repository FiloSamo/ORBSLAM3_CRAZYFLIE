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

TOPIC_IMG = '/cam0/image_raw'
ODOMETRY_TOPIC = '/orb_slam3/odom'

## Debug
FPS_FILE = 'fps.txt'

# Custom CPX command
WIFI_POSITION_SENDED = 0x40  # Must match the value in the C code

class CrazyflieIMGNode(Node):

    def __init__(self):
        super().__init__('crazyflie_img_node')

        self.declare_parameter("ip", "192.168.4.1")
        self.declare_parameter("port", 5000)
        #self.declare_parameter("file_log", False)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        #self.LOG_ACTIVE = self.get_parameter("file_log").get_parameter_value().bool_value

        self.img_count = 0
        self.pose_count = 0
        self.time = 0

        self.callback_group = ReentrantCallbackGroup()
        self.mutex = Lock()

        # Publisher
        self.publisher_ = self.create_publisher(ImageAndInt, TOPIC_IMG, 50) # Puslishing on the topic
        self.subscriber = self.create_subscription(Odometry, ODOMETRY_TOPIC, self.odom_callback, 50, callback_group=self.callback_group)
        self.routine = self.create_timer(0.01, self.read_data, callback_group=self.callback_group)

        self.bridge = CvBridge()

        deck_ip, deck_port = ip , port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        self.get_logger().info(f"Connected to {deck_ip}:{deck_port}")

        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        self.get_logger().info('Connecting to Crazyflie...')

        ## Debug
        with open(FPS_FILE, 'w') as f:
            f.write('')
    
    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            packet = self.client_socket.recv(size - len(data))
            if not packet:
                raise RuntimeError("Connection lost")
            data.extend(packet)
        return data
    

    def build_cpx_packet(self, rsv, version, function, source, destination, last_packet, data):

        length = 2 + len(data)  # 2 bytes for length + data length
        length_bytes = struct.pack('<H', length)

        fmt = 'u1u1u3u3u2u6'
        # header fields
        header = bitstruct.pack(fmt, rsv, last_packet, source, destination, version, function)

        return length_bytes + header + data

    def send_position(self, client_socket, x, y, z, roll, pitch, yaw, timestamp):

        payload = struct.pack('>B6fI', WIFI_POSITION_SENDED, x, y, z, roll, pitch, yaw, timestamp)
        
        # Build the CPX packet
        full_packet = self.build_cpx_packet(rsv=0,
                                             version=0,
                                             function=5,  # CPX_F_APP
                                             source=3,    # CPX_T_WIFI_HOST
                                             destination=4,  # CPX_T_GAP8
                                             last_packet=0,    # bool → 1 byte
                                             data=payload)

        # Send the packet
        client_socket.sendall(full_packet)

    def odom_callback(self, msg):
        # Extract position and orientation from the Odometry message
        if self.pose_count == 0:
            self.img_count = 1
        self.pose_count += 1

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = float(position.x)
        y = float(position.y)
        z = float(position.z)
        roll = float(orientation.x)
        pitch = float(orientation.y)
        yaw = float(orientation.z)
        timestamp = int(msg.header.stamp.sec)
        # Send the position data to the Crazyflie
        with self.mutex:
            self.send_position(self.client_socket, x, y, z, roll, pitch, yaw, timestamp)

        
    def read_data(self):
        try:
            with self.mutex:
                packet_info = self.rx_bytes(4)
                length, routing, function = struct.unpack('<HBB', packet_info)
                header = self.rx_bytes(length - 2)
                magic, width, height, depth, fmt, size = struct.unpack('<BHHBBI', header)
                if magic != 0xBC:
                    return 0

                img_stream = bytearray()
            
                while len(img_stream) < size:
                    chunk_info = self.rx_bytes(4)
                    chunk_len, dst, src = struct.unpack('<HBB', chunk_info)
                    chunk = self.rx_bytes(chunk_len - 2)
                    img_stream.extend(chunk)
            
                # Read the time from the stream
            
                packetInfoRaw = self.rx_bytes(4)
                [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
                timestamp_data = self.rx_bytes(length - 2)

            timestamp = np.frombuffer(timestamp_data, dtype=np.uint32)[0]  

            #frame_queue.put((img_stream, fmt), timeout=1)
            # Publish the image  
            self.img_publish(img_stream, int(timestamp), fmt)  
            self.img_count += 1
            second , nsecond  = self.get_clock().now().seconds_nanoseconds()
            n_time = second + nsecond*1e-9
            fps = int(1 / (n_time - self.time) if n_time - self.time != 0 else 0)
            self.get_logger().info('Image number: %d timestamp: %d fps: %d pose number: %d' % (self.img_count, timestamp, fps, self.pose_count))
            # Debug FPS
            with open(FPS_FILE, 'a') as f:
                f.write(f"{fps}\n")

            self.time = n_time
        except Exception as e:
            self.get_logger().error(f"[Receiver Error] {e}")
            return 0

    def img_publish(self, img_stream, timestamp, fmt):
        # Create a new Image message
        msg = ImageAndInt()
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
