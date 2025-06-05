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
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('duck_config_file', '')

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        cfg_file = self.get_parameter('duck_config_file').get_parameter_value().string_value or None



        # instantiate HWI
        duck_cfg = DuckConfig(cfg_file) if cfg_file else DuckConfig()
        self.hwi = HWI(duck_cfg)

        self.dummy_joints = ["left_antenna", "right_antenna"]  # Dummy joints for urdf viz. Not controlled by hardware.
        self.dummy_joint_values = [0.0, 0.0]
        self.dummy_joint_insert_idx = 9
        joint_names = list(self.hwi.joints.keys())
        joint_names[self.dummy_joint_insert_idx:self.dummy_joint_insert_idx] = self.dummy_joints
        self.joint_names = joint_names

        self.hwi.turn_on()
        # Initialize target_positions as a dict { joint_name: init_position }
        self.target_positions = {name: self.hwi.init_pos[name] for name in self.hwi.joints.keys()}
        # Also ensure dummy joints are in the dict:
        # self.target_positions["left_antenna"] = 0.0
        # self.target_positions["right_antenna"] = 0.0

        # publisher & timer
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber for target joint states
        # self.target_positions = None
        self.create_subscription(
            JointState,
            '/target_joint_states',
            self.target_callback,
            10
        )

        # Timer at 100 Hz for both sending targets and publishing actuals
        self.timer = self.create_timer(1.0 / rate, self.update_callback)

        self.get_logger().info(f'Publishing joint_states at {rate:.1f} Hz')

    def target_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.dummy_joints:
                continue
            if name in self.target_positions:
                self.target_positions[name] = pos
            else:
                self.get_logger().warn("Received target for unknown joint: %s", name)

        # store latest target positions as a dict {name: position}
        # self.target_positions = dict(zip(msg.name, msg.position))

    def update_callback(self):
        # 1) Send target positions to HWI, if available
        if self.target_positions:
            try:
                self.hwi.set_position_all(self.target_positions)
            except Exception as e:
                self.get_logger().warn(f'Failed to send targets: {e}')

        # 2) Read present positions & velocities
        positions = self.hwi.get_present_positions()
        velocities = self.hwi.get_present_velocities()

        if positions is None or velocities is None:
            self.get_logger().warn('HWI read failure, skipping publish')
            return

        # 3) Publish actual joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Convert positions and velocities to lists and insert dummy joint values
        positions_list = positions.tolist()
        velocities_list = velocities.tolist()

        positions_list[self.dummy_joint_insert_idx:self.dummy_joint_insert_idx] = self.dummy_joint_values
        velocities_list[self.dummy_joint_insert_idx:self.dummy_joint_insert_idx] = self.dummy_joint_values

        msg.name = self.joint_names
        msg.position = positions_list
        msg.velocity = velocities_list

        self.pub.publish(msg)

    def destroy_node(self):
        self.hwi.turn_off()
        super().destroy_node()
        self.get_logger().info('Shutting down joint_state_publisher_node')
        # any cleanup your HWI needs can go here

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
