#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import socket
import threading

class VelocityComputationNode(Node):
    def __init__(self):
        super().__init__('velocity_computation_node')
        self.publisher = self.create_publisher(Float32MultiArray, '/wheel_velocities', 10)

        # UDP setup
        self.udp_ip = "0.0.0.0"  # Listen on all available interfaces
        self.udp_port = 12345
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        # Start UDP listener thread
        self.udp_thread = threading.Thread(target=self.udp_listener)
        self.udp_thread.daemon = True
        self.udp_thread.start()

    def udp_listener(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            command = data.decode('utf-8')
            self.process_command(command)

    def process_command(self, command):
        linear_velocity = 0.0
        angular_velocity = 0.0

        if command == 'w':
            linear_velocity = 0.25
        elif command == 's':
            linear_velocity = -0.25
        elif command == 'a':
            angular_velocity = 3
        elif command == 'd':
            angular_velocity = -3
        elif command == 'x':
            linear_velocity = 0.0
            angular_velocity = 0.0

        left_speed, right_speed = self.compute_wheel_speeds(linear_velocity, angular_velocity)
        self.publish_wheel_speeds(left_speed, right_speed)

    def compute_wheel_speeds(self, linear_velocity, angular_velocity):
        # Kinematic parameters
        wheel_base = 0.14  # distance between wheels
        wheel_radius = 0.03125  # radius of the wheels

        v_left = (2 * linear_velocity - angular_velocity * wheel_base) / (2 * wheel_radius)
        v_right = (2 * linear_velocity + angular_velocity * wheel_base) / (2 * wheel_radius)

        return v_left, v_right

    def publish_wheel_speeds(self, left_speed, right_speed):
        msg = Float32MultiArray()
        msg.data = [left_speed, right_speed]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityComputationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()