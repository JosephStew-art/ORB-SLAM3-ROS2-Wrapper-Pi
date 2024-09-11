#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import RPi.GPIO as GPIO

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

    def velocity_callback(self, msg):
        left_speed, right_speed = msg.data
        self.set_motor_speeds(left_speed, right_speed)

    def encoder_callback(self, msg):
        new_left_ticks, new_right_ticks = msg.data
        
        # Calculate the change in ticks
        left_diff = new_left_ticks - self.prev_left_ticks
        right_diff = new_right_ticks - self.prev_right_ticks

        # Update ticks based on direction
        self.left_ticks += left_diff if self.left_direction >= 0 else -left_diff
        self.right_ticks += right_diff if self.right_direction >= 0 else -right_diff

        # Update previous tick values
        self.prev_left_ticks = new_left_ticks
        self.prev_right_ticks = new_right_ticks
        
        # Print updated tick values
        self.get_logger().info(f'Left ticks: {self.left_ticks}, Right ticks: {self.right_ticks}')

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