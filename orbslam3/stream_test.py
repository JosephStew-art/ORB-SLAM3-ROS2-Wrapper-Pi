#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.listener_callback,
            10)
        self.cv_bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert compressed image to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow("Camera Feed", image)
        cv2.waitKey(1)  # Refresh the image display

        # Print the timestamp
        stamp = msg.header.stamp
        print(f"Frame timestamp: {stamp.sec}.{stamp.nanosec:09d}")

def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    
    try:
        rclpy.spin(compressed_image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        compressed_image_subscriber.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()