import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import RPi.GPIO as GPIO
import time
import math

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.reset()

    def reset(self):
        self.p_term = 0
        self.i_term = 0
        self.d_term = 0
        self.last_error = 0
        self.last_time = time.time()

    def update(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        error = setpoint - measured_value

        self.p_term = self.kp * error
        self.i_term += self.ki * error * dt
        self.d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0

        output = self.p_term + self.i_term + 0

        if self.output_limits:
            output = max(min(output, self.output_limits[1]), self.output_limits[0])

        self.last_error = error
        self.last_time = current_time

        return output

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
        self.wheel_radius = 0.03125  # meters
        self.wheel_base = 0.14  # meters (distance between wheels)
        self.left_last_tick_time = self.get_clock().now()
        self.right_last_tick_time = self.get_clock().now()
        self.left_angular_velocity = 0.0
        self.right_angular_velocity = 0.0

        # Desired velocities (angular)
        self.desired_left_angular_velocity = 0.0
        self.desired_right_angular_velocity = 0.0

        # PID controllers
        self.left_pid = PIDController(kp=0.1, ki=0.1, kd=0.05, output_limits=(-100, 100))
        self.right_pid = PIDController(kp=0.1, ki=0.1, kd=0.05, output_limits=(-100, 100))
        self.heading_pid = PIDController(kp=10, ki=2, kd=0.05, output_limits=(-0.5, 0.5))

        # Control loop timer
        self.control_loop_rate = 0.05  # 20 Hz
        self.create_timer(self.control_loop_rate, self.control_loop)

        # Flag to determine if PID control should be used
        self.use_pid_control = False

    def velocity_callback(self, msg):
        self.desired_left_angular_velocity, self.desired_right_angular_velocity = msg.data
        
        # Check if we should use PID control
        self.use_pid_control = (self.desired_left_angular_velocity > 0 and 
                                self.desired_right_angular_velocity > 0 and 
                                abs(self.desired_left_angular_velocity - self.desired_right_angular_velocity) < 1e-6)
        
        self.get_logger().info(f'Desired angular velocities: Left: {self.desired_left_angular_velocity:.2f} rad/s, Right: {self.desired_right_angular_velocity:.2f} rad/s')
        self.get_logger().info(f'Using PID control: {self.use_pid_control}')

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

    def control_loop(self):
        if self.use_pid_control:
            # PID control for forward motion
            left_output = self.left_pid.update(self.desired_left_angular_velocity, self.left_angular_velocity)
            right_output = self.right_pid.update(self.desired_right_angular_velocity, self.right_angular_velocity)

            # Calculate heading error (difference in distances traveled by each wheel)
            left_distance = self.left_ticks * (2 * math.pi * self.wheel_radius) / self.ticks_per_rotation
            right_distance = self.right_ticks * (2 * math.pi * self.wheel_radius) / self.ticks_per_rotation
            heading_error = left_distance - right_distance
            
            heading_correction = self.heading_pid.update(0, heading_error)

            # Apply heading correction
            #left_output -= heading_correction
            #right_output += heading_correction

            # Convert PID output to PWM duty cycle (assuming PID output range is -100 to 100)
            left_pwm = abs(left_output)
            right_pwm = abs(right_output)
        else:
            # Open-loop control for other motions
            # Here we're assuming the desired angular velocities are proportional to the required PWM duty cycle
            # You might need to adjust this conversion based on your robot's characteristics
            max_angular_velocity = 10.0  # This should be set to your robot's maximum angular velocity
            left_pwm = abs(self.desired_left_angular_velocity) / max_angular_velocity * 100
            right_pwm = abs(self.desired_right_angular_velocity) / max_angular_velocity * 100

        # Set motor speeds
        self.set_motor_speeds(left_pwm, right_pwm)

    def set_motor_speeds(self, left_speed, right_speed):
        # Set left motor direction
        if self.desired_left_angular_velocity > 0:
            GPIO.output(self.gpio_pins_forward_left, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
            self.left_direction = 1
        elif self.desired_left_angular_velocity < 0:
            GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.HIGH)
            self.left_direction = -1
        else:
            GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
            self.left_direction = 0

        # Set right motor direction
        if self.desired_right_angular_velocity > 0:
            GPIO.output(self.gpio_pins_forward_right, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
            self.right_direction = 1
        elif self.desired_right_angular_velocity < 0:
            GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.HIGH)
            self.right_direction = -1
        else:
            GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
            self.right_direction = 0

        # Set PWM duty cycle based on speed (0 to 100)
        self.pwm_left.ChangeDutyCycle(min(abs(left_speed), 100))
        self.pwm_right.ChangeDutyCycle(min(abs(right_speed), 100))

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()