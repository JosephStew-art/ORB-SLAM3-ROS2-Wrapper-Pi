#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        self.br = CvBridge()

    def image_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
        cv2.imshow("Camera Stream", current_frame)
        cv2.waitKey(1)
        self.get_logger().info('Received video frame')

def main(args=None):
    rclpy.init(args=args)
    image_viewer = ImageViewerNode()
    rclpy.spin(image_viewer)
    image_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()