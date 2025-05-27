import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import math

import argparse
import socket
import struct
import threading
import queue
import cv2
import numpy as np
import time

TOPIC_IMG = '/cam0/image_raw'


class CrazyflieIMGNode(Node):

    def __init__(self):
        super().__init__('crazyflie_img_node')

        self.declare_parameter("ip", "192.168.4.1")
        self.declare_parameter("port", 5000)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

        # Publisher
        self.publisher_ = self.create_publisher(Image, TOPIC_IMG, 50) # Puslishing on the topic
        self.bridge = CvBridge()

        deck_ip, deck_port = ip , port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print(f"Connected to {deck_ip}:{deck_port}")

        frame_queue = queue.Queue(maxsize=10)

        self.get_logger().info('Connecting to Crazyflie...')
    
    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            packet = self.client_socket.recv(size - len(data))
            if not packet:
                raise RuntimeError("Connection lost")
            data.extend(packet)
        return data
        
    def read_data(self):
        count = 0
        time = 0
        while True:
            self.get_logger().info("Sto eseguendo")
            try:
                packet_info = self.rx_bytes(4)
                length, routing, function = struct.unpack('<HBB', packet_info)
                header = self.rx_bytes(length - 2)
                magic, width, height, depth, fmt, size = struct.unpack('<BHHBBI', header)
                if magic != 0xBC:
                    continue

                img_stream = bytearray()
                while len(img_stream) < size:
                    chunk_info = self.rx_bytes(4)
                    chunk_len, dst, src = struct.unpack('<HBB', chunk_info)
                    chunk = self.rx_bytes(chunk_len - 2)
                    img_stream.extend(chunk)

                #frame_queue.put((img_stream, fmt), timeout=1)
                # Publish the image
                self.img_publish(img_stream, fmt)
                count += 1
                self.get_logger().info('count: %d' % count)
                second , nsecond  = self.get_clock().now().seconds_nanoseconds()
                n_time = second + nsecond*1e-9
                self.get_logger().info('fps: {}'.format(1/(float(n_time) - float(time))))
                time = n_time
            except Exception as e:
                print(f"[Receiver Error] {e}")
                break
    
    def img_publish(self, img_stream, fmt):
        # Create a new Image message
        img_msg = Image()

        if fmt == 0:
            raw = np.frombuffer(img_stream, dtype=np.uint8).reshape((244, 324))
        else:
            arr = np.frombuffer(img_stream, np.uint8)
            raw = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)

        #img = cv2.rotate(raw, cv2.ROTATE_180)
        img = raw
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "cam0"

        # Publish the image
        self.publisher_.publish(img_msg)


def main():

    rclpy.init()
    node = CrazyflieIMGNode()
    node.read_data()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
