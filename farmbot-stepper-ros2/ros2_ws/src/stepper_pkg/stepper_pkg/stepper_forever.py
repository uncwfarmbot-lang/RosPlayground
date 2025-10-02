#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

DIR_PIN = 16
STEP_PIN = 12

class StepperForever(Node):
    def __init__(self):
        super().__init__('stepper_forever')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(STEP_PIN, GPIO.OUT)
        GPIO.output(DIR_PIN, GPIO.HIGH)
        self.get_logger().info("Stepper running forward... Ctrl+C to stop.")
        while rclpy.ok():
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(0.001)

def main(args=None):
    rclpy.init(args=args)
    try:
        StepperForever()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
