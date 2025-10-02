#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

DIR_PIN = 16
STEP_PIN = 12

class StepperVariable(Node):
    def __init__(self):
        super().__init__('stepper_variable')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(STEP_PIN, GPIO.OUT)

        self.declare_parameter("steps", 200)
        self.declare_parameter("direction", "forward")

        steps = self.get_parameter("steps").value
        direction = self.get_parameter("direction").value

        GPIO.output(DIR_PIN, GPIO.HIGH if direction == "forward" else GPIO.LOW)
        self.get_logger().info(f"Moving {steps} steps {direction}")

        for _ in range(steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(0.001)

        self.get_logger().info("Stepper movement complete.")

def main(args=None):
    rclpy.init(args=args)
    try:
        StepperVariable()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
