#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageResolutionChecker(Node):
    def __init__(self):
        super().__init__('image_resolution_checker')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Decode the compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Get the image dimensions
        height, width, _ = image.shape
        
        # Print the resolution
        self.get_logger().info(f'Image Resolution: {width}x{height}')
        
        # Optionally, you can also print the encoding format
        self.get_logger().info(f'Encoding format: {msg.format}')

def main(args=None):
    rclpy.init(args=args)
    image_resolution_checker = ImageResolutionChecker()
    rclpy.spin(image_resolution_checker)
    image_resolution_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()