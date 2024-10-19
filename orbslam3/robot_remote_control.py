#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class NormalizedCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('normalized_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Linear: {linear_x}, Angular: {angular_z}')

def on_press(key, node):
    try:
        if key.char == 'w':
            node.publish_cmd_vel(1.0, 0.0)
        elif key.char == 's':
            node.publish_cmd_vel(-1.0, 0.0)
        elif key.char == 'a':
            node.publish_cmd_vel(0.0, 1.0)
        elif key.char == 'd':
            node.publish_cmd_vel(0.0, -1.0)
        elif key.char == 'x':
            node.publish_cmd_vel(0.0, 0.0)
    except AttributeError:
        pass

def main(args=None):
    rclpy.init(args=args)
    node = NormalizedCmdVelPublisher()

    listener = keyboard.Listener(on_press=lambda key: on_press(key, node))
    listener.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()