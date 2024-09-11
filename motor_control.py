#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import RPi.GPIO as GPIO
import time
import math

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            Float32MultiArray, 
            '/wheel_velocities', 
            self.velocity_callback, 
            10
        )
        self.encoder_subscription = self.create_subscription(
            Int32MultiArray, 
            'encoder_ticks', 
            self.encoder_callback, 
            10
        )

        # GPIO pin setup
        self.gpio_pins_forward_right = 17
        self.gpio_pins_forward_left = 27
        self.gpio_pins_backwards_right = 5
        self.gpio_pins_backwards_left = 6

        # GPIO pin setup for PWM control
        self.pwm_pin_left = 18
        self.pwm_pin_right = 19

        # GPIO pins setup for PWM speed control
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in [self.gpio_pins_forward_right, self.gpio_pins_forward_left, 
                    self.gpio_pins_backwards_right, self.gpio_pins_backwards_left]:
            GPIO.setup(pin, GPIO.OUT)

        GPIO.setup(self.pwm_pin_left, GPIO.OUT)
        GPIO.setup(self.pwm_pin_right, GPIO.OUT)

        # Initialize PWM on the GPIO pins
        self.pwm_left = GPIO.PWM(self.pwm_pin_left, 100)  # 100 Hz frequency
        self.pwm_right = GPIO.PWM(self.pwm_pin_right, 100)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Set initial state to stop
        GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
        GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)

        # Variables to track wheel direction and ticks
        self.left_direction = 0  # 0: stopped, 1: forward, -1: backward
        self.right_direction = 0
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Timing and velocity calculation variables
        self.ticks_per_rotation = 10
        self.left_last_tick_time = self.get_clock().now()
        self.right_last_tick_time = self.get_clock().now()
        self.left_angular_velocity = 0.0
        self.right_angular_velocity = 0.0

    def velocity_callback(self, msg):
        left_speed, right_speed = msg.data
        self.set_motor_speeds(left_speed, right_speed)
        self.get_logger().info(f'Desired angular velocities: Left: {left_speed:.2f} rad/s, Right: {right_speed:.2f} rad/s')

    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        new_left_ticks, new_right_ticks = msg.data

        # Update left wheel
        left_tick_diff = new_left_ticks - self.prev_left_ticks
        if left_tick_diff != 0:
            dt = (current_time - self.left_last_tick_time).nanoseconds / 1e9
            self.left_angular_velocity = self.calculate_angular_velocity(abs(left_tick_diff), dt, self.left_direction)
            self.left_last_tick_time = current_time
            self.left_ticks += left_tick_diff if self.left_direction >= 0 else -left_tick_diff
            self.prev_left_ticks = new_left_ticks
            self.get_logger().info(f'Left ticks: {self.left_ticks}, Angular velocity: {self.left_angular_velocity:.2f} rad/s')

        # Update right wheel
        right_tick_diff = new_right_ticks - self.prev_right_ticks
        if right_tick_diff != 0:
            dt = (current_time - self.right_last_tick_time).nanoseconds / 1e9
            self.right_angular_velocity = self.calculate_angular_velocity(abs(right_tick_diff), dt, self.right_direction)
            self.right_last_tick_time = current_time
            self.right_ticks += right_tick_diff if self.right_direction >= 0 else -right_tick_diff
            self.prev_right_ticks = new_right_ticks
            self.get_logger().info(f'Right ticks: {self.right_ticks}, Angular velocity: {self.right_angular_velocity:.2f} rad/s')

    def calculate_angular_velocity(self, ticks, dt, direction):
        rotations = ticks / self.ticks_per_rotation
        angular_velocity = (rotations * 2 * math.pi) / dt  # rad/s
        return angular_velocity * (1 if direction >= 0 else -1)


    def set_motor_speeds(self, left_speed, right_speed):
        # Set left motor direction
        if left_speed > 0:
            GPIO.output(self.gpio_pins_forward_left, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
            self.left_direction = 1
        elif left_speed < 0:
            GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.HIGH)
            self.left_direction = -1
        else:
            GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
            self.left_direction = 0

        # Set right motor direction
        if right_speed > 0:
            GPIO.output(self.gpio_pins_forward_right, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
            self.right_direction = 1
        elif right_speed < 0:
            GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.HIGH)
            self.right_direction = -1
        else:
            GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
            self.right_direction = 0

        # Set PWM duty cycle based on speed (0 to 100)
        self.pwm_left.ChangeDutyCycle(abs(left_speed))
        self.pwm_right.ChangeDutyCycle(abs(right_speed))

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()