#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import socket
import struct
import threading
import queue
import cv2
import numpy as np
import time

parser = argparse.ArgumentParser(description='Fast AI-deck JPEG streamer')
parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_ip, deck_port = args.n, args.p
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((deck_ip, deck_port))
print(f"Connected to {deck_ip}:{deck_port}")

def rx_bytes(size):
    data = bytearray()
    while len(data) < size:
        packet = client_socket.recv(size - len(data))
        if not packet:
            raise RuntimeError("Connection lost")
        data.extend(packet)
    return data

frame_queue = queue.Queue(maxsize=10)

def receive_thread():
    while True:
        try:
            packet_info = rx_bytes(4)
            length, routing, function = struct.unpack('<HBB', packet_info)
            header = rx_bytes(length - 2)
            magic, width, height, depth, fmt, size = struct.unpack('<BHHBBI', header)
            if magic != 0xBC:
                continue

            img_stream = bytearray()
            while len(img_stream) < size:
                chunk_info = rx_bytes(4)
                chunk_len, dst, src = struct.unpack('<HBB', chunk_info)
                chunk = rx_bytes(chunk_len - 2)
                img_stream.extend(chunk)

            frame_queue.put((img_stream, fmt), timeout=1)
        except Exception as e:
            print(f"[Receiver Error] {e}")
            break

def display_thread():
    count = 0
    start = time.time()
    while True:
        try:
            img_stream, fmt = frame_queue.get(timeout=2)
            count += 1
            mean_time = (time.time() - start) / count
            print(f"FPS: {1/mean_time:.2f}")

            if fmt == 0:
                raw = np.frombuffer(img_stream, dtype=np.uint8).reshape((244, 324))
                cv2.imshow('Raw', raw)
            else:
                arr = np.frombuffer(img_stream, np.uint8)
                decoded = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
                if decoded is not None:
                    cv2.imshow('JPEG', decoded)
                    if args.save:
                        cv2.imwrite(f"stream_out/img_{count:06d}.jpg", decoded)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except queue.Empty:
            continue

threading.Thread(target=receive_thread, daemon=True).start()
display_thread()