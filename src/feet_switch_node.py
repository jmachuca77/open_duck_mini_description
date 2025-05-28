#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from open_duck_mini_description.msg import FeetState

import board
import digitalio
import numpy as np

LEFT_FOOT_PIN = board.D22
RIGHT_FOOT_PIN = board.D27

class FeetContacts:
    def __init__(self):
        self.left_foot = digitalio.DigitalInOut(LEFT_FOOT_PIN)
        self.left_foot.direction = digitalio.Direction.INPUT
        self.left_foot.pull = digitalio.Pull.UP

        self.right_foot = digitalio.DigitalInOut(RIGHT_FOOT_PIN)
        self.right_foot.direction = digitalio.Direction.INPUT
        self.right_foot.pull = digitalio.Pull.UP

    def get(self):
        left = not self.left_foot.value
        right = not self.right_foot.value
        return np.array([left, right])

    def __del__(self):
        self.left_foot.deinit()
        self.right_foot.deinit()

class FeetSwitchNode(Node):
    def __init__(self):
        super().__init__('feet_switch_node')
        self.contacts = FeetContacts()
        self.pub = self.create_publisher(FeetState, 'feet_switch', 1)
        self.timer = self.create_timer(1.0 / 50.0, self.timer_cb)
        self.get_logger().info('Started FeetSwitchNode @50Hz')

    def timer_cb(self):
        arr = self.contacts.get()
        left, right = bool(arr[0]), bool(arr[1])
        # bit 0 = left, bit 1 = right
        msg = FeetState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.left_contact = left
        msg.right_contact = right
        self.pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        del self.contacts  # triggers cleanup

def main(args=None):
    rclpy.init(args=args)
    node = FeetSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
