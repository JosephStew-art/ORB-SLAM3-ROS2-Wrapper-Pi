#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import requests
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()
        self.video_url = 'http://192.168.68.50:8080/video_feed'  # Replace <raspberry_pi_ip> with your Pi's IP address

    def timer_callback(self):
        try:
            response = requests.get(self.video_url, stream=True, timeout=1)
            if response.status_code == 200:
                bytes_data = bytes()
                for chunk in response.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        # Convert to ROS Image message
                        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                        # Set the current time as the timestamp
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher_.publish(msg)
                        break  # Exit after publishing one frame
            else:
                self.get_logger().warn(f'Failed to connect to the video stream. Status code: {response.status_code}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error connecting to the video stream: {e}')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()