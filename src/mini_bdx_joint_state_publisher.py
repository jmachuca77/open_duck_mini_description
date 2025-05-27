#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.duck_config import DuckConfig

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('mini_bdx_joint_state_publisher')

        # parameters
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('duck_config_file', '')

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        cfg_file = self.get_parameter('duck_config_file').get_parameter_value().string_value or None

        # instantiate HWI
        duck_cfg = DuckConfig(cfg_file) if cfg_file else DuckConfig()
        self.hwi = HWI(duck_cfg)

        # publisher & timer
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_callback)

        self.get_logger().info(f'Publishing joint_states at {rate:.1f} Hz')

    def publish_callback(self):
        positions = self.hwi.get_present_positions()
        velocities = self.hwi.get_present_velocities()

        if positions is None or velocities is None:
            self.get_logger().warn('HWI read failure, skipping this cycle')
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.hwi.joints.keys())
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Shutting down joint_state_publisher_node')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
