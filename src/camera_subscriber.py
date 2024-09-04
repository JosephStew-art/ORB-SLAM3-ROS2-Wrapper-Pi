#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            1)
        self.bridge = CvBridge()
        self.window_name = "Resized Camera Feed"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)  # Set the window size to 640x480

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return

        # Resize the image
        resized_image = cv2.resize(cv_image, (640, 480))  # Resize to 640x480

        cv2.imshow(self.window_name, resized_image)
        cv2.waitKey(1)  # Wait for a short time to update the window

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        camera_subscriber.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()