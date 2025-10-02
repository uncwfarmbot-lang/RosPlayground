#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import String

DIR_PIN = 16
STEP_PIN = 12

class StepperMotor(Node):
    def __init__(self):
        super().__init__('stepper_motor')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(STEP_PIN, GPIO.OUT)
        self.sub = self.create_subscription(String, 'motor_command', self.command_callback, 10)
        self.get_logger().info("Stepper motor node ready. Waiting for commands...")

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        if command == "forward":
            GPIO.output(DIR_PIN, GPIO.HIGH)
            self.run_steps(200)
        elif command == "backward":
            GPIO.output(DIR_PIN, GPIO.LOW)
            self.run_steps(200)
        else:
            self.get_logger().warn("Unknown command. Use 'forward' or 'backward'.")

    def run_steps(self, steps):
        for _ in range(steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = StepperMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
