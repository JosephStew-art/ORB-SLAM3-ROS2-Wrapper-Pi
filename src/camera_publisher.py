#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 1)
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Resize the frame to 100x100 pixels
            resized_frame = cv2.resize(frame, (640, 480))
            
            msg = self.bridge.cv2_to_imgmsg(resized_frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()  # Set the current time as the timestamp
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()