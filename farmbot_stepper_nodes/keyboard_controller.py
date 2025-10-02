#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, termios, tty

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("Keyboard controller ready. Press 'w'=forward, 's'=backward, 'q'=quit.")
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            msg = String()
            if key == 'w':
                msg.data = 'forward'
                self.publisher.publish(msg)
                self.get_logger().info("Sent: forward")
            elif key == 's':
                msg.data = 'backward'
                self.publisher.publish(msg)
                self.get_logger().info("Sent: backward")
            elif key == 'q':
                self.get_logger().info("Exiting controller.")
                break

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
