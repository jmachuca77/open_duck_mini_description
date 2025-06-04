#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import onnxruntime as ort
import numpy as np
import time
import os
from collections import deque

# ROS2 message types
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from open_duck_mini_description.msg import FeetState


def quat_rotate_inverse(q, v):
    q = np.array(q)
    v = np.array(v)

    q_w = q[-1]
    q_vec = q[:3]

    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * (np.dot(q_vec, v)) * 2.0

    return a - b + c


class WalkPolicyInferenceNode(Node):
    """Walk policy inference node that subscribes to sensor data and publishes target joint commands."""
    def __init__(self):
        super().__init__('walk_policy_inference_node')

        # Load the ONNX model from the package assets directory
        pkg_share = get_package_share_directory('open_duck_mini_description')
        model_path = os.path.join(pkg_share, 'checkpoints', 'BEST_WALK_ONNX_2.onnx')
        self.model = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])

        # Setup joint names and policy parameters
        self.setup_policy_params()

        # Initialize publishers
        self.target_joint_states_pub = self.create_publisher(JointState, '/target_joint_states', 1)

        # Initialize subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.feet_contact_sub = self.create_subscription(FeetState, '/feet_switch', self.feet_contact_callback, 1)
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/bno055/imu_raw', self.imu_callback, 1)

        # Initialize sensor data variables
        self.cmd_vel = None
        self.feet_contact = None
        self.joint_positions = None
        self.joint_velocities = None
        self.projected_gravity = None
        self.angular_velocity = None

        # Timestamps for tracking sensor updates
        self.last_imu_update = None
        self.last_joint_states_update = None
        self.last_cmd_vel_update = None
        self.last_feet_contact_update = None
        self.last_inference_time = None

        # Initialize history arrays
        self.obs_history = np.zeros((self.obs_history_length, self.obs_size))  # Adjust size as needed
        self.action_history = np.zeros((self.action_history_length, len(self.joint_names)))

        # For rate logging
        self.last_rate_log_time = time.time()

        self.init_pos = np.array([
                0.002, 0.053, -0.63, 1.368, -0.784, 
                0.0, 0, 0, 0, 0, 0, 
                -0.003, -0.065, 0.635, 1.379, -0.796,
            ])

        self.get_logger().info("Walk Policy Inference Node initialized.")

        # Create a timer to run the policy loop at 50 Hz
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.started = False

    def setup_policy_params(self):
        """Setup policy parameters and joint information."""
        # Policy parameters
        self.joint_pos_scale = 1.0
        self.joint_vel_scale = 1.0
        self.angular_vel_scale_obs = 1.0
        self.angular_vel_scale = 0.25
        self.obs_history_length = 1
        self.action_history_length = 3
        self.obs_size = 40

        self.action_clip = [-5.0, 5.0]
        self.obs_clip = [-5.0, 5.0]

        self.power_scale = 1.5
        self.lin_vel_x_range = [-0.3, 0.3]
        self.lin_vel_y_range = [-0.3, 0.3]
        self.yaw_range = [-0.3, 0.3]

        # Joint names and masking
        self.joint_names = [
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", 
            "left_knee", "left_ankle", "neck_pitch", "head_pitch", 
            "head_yaw", "head_roll", "left_antenna", "right_antenna",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"
        ]

        self.mask_joints = ['neck_pitch', 'head_pitch', 'head_yaw', "head_roll", "left_antenna", "right_antenna"]
        self.mask_joint_idx = np.array([self.joint_names.index(joint) for joint in self.mask_joints])

    def imu_callback(self, msg):
        """Process incoming IMU data."""
        quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.projected_gravity = quat_rotate_inverse(quat, [0, 0, -1.0])
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.last_imu_update = self.get_clock().now()

    def scale_cmd_vel(self, linear_x, linear_y, angular_z):
        """Scale command velocities to their respective ranges."""
        lin_vel_x = np.clip(linear_x, -1, 1) * (self.lin_vel_x_range[1] - self.lin_vel_x_range[0]) / 2 + (
            self.lin_vel_x_range[1] + self.lin_vel_x_range[0]) / 2
        lin_vel_y = np.clip(linear_y, -1, 1) * (self.lin_vel_y_range[1] - self.lin_vel_y_range[0]) / 2 + (
            self.lin_vel_y_range[1] + self.lin_vel_y_range[0]) / 2
        yaw = np.clip(angular_z, -1, 1) * (self.yaw_range[1] - self.yaw_range[0]) / 2 + (
            self.yaw_range[1] + self.yaw_range[0]) / 2
        return np.array([lin_vel_x, lin_vel_y, yaw * self.angular_vel_scale])

    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands."""
        self.cmd_vel = self.scale_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_vel_update = self.get_clock().now()

    def feet_contact_callback(self, msg):
        """Process incoming feet contact data."""
        left_foot = msg.left_contact  # Extract left foot status (bit 0)
        right_foot = msg.right_contact  # Extract right foot status (bit 1)
        self.feet_contact = np.array([left_foot, right_foot])
        self.last_feet_contact_update = self.get_clock().now()

    def joint_states_callback(self, msg):
        """Process incoming joint states."""
        self.joint_positions = np.array(msg.position)
        self.joint_velocities = np.array(msg.velocity)
        self.last_joint_states_update = self.get_clock().now()

    def timer_callback(self):
        """Timer callback to run inference at 50 Hz."""
        if not self.started:
            # Wait until all required data is available
            if any(x is None for x in [
                self.cmd_vel,
                self.feet_contact,
                self.joint_positions,
                self.joint_velocities,
                self.projected_gravity,
                self.angular_velocity,
                self.last_imu_update,
                self.last_joint_states_update,
                self.last_cmd_vel_update,
                self.last_feet_contact_update
            ]):
                now = time.time()
                if now - self.last_rate_log_time > 1.0:
                    self.get_logger().info("Waiting for all policy inputs to be available...")
                    self.last_rate_log_time = now
                return

            self.get_logger().info("All sensor data received. Starting policy execution.")
            self.last_inference_time = self.get_clock().now()

            # Optional interactive start; if running in a headless environment, you may remove this
            input("Press Enter to start the control loop...")
            self.started = True
            return

        # Once started, enforce fresh data requirement
        now_ros = self.get_clock().now()
        all_data_fresh = (
            self.last_imu_update > self.last_inference_time and
            self.last_joint_states_update > self.last_inference_time and
            self.last_cmd_vel_update > self.last_inference_time and
            self.last_feet_contact_update > self.last_inference_time
        )
        if not all_data_fresh:
            return

        # Record the time of this inference cycle
        self.last_inference_time = now_ros

        # Prepare the input for the model
        obs = np.concatenate([
            self.projected_gravity,
            self.joint_positions * self.joint_pos_scale,
            self.joint_velocities * self.joint_vel_scale,
            self.angular_velocity * self.angular_vel_scale_obs,
            self.feet_contact
        ])

        # Update observation history
        self.obs_history[1:, :] = self.obs_history[:-1, :].copy()
        self.obs_history[0, :len(obs)] = obs

        # Combine with action history and cmd_vel
        input_data = np.concatenate([
            self.obs_history.flatten(),
            self.action_history.flatten(),
            self.cmd_vel
        ]).reshape(1, -1)

        input_data = np.clip(input_data, self.obs_clip[0], self.obs_clip[1])

        # Run the model
        outputs = self.model.run(None, {'obs': input_data.astype(np.float32)})

        # Extract and process actions
        actions = outputs[0].flatten()
        actions = np.clip(actions, self.action_clip[0], self.action_clip[1])

        # Update action history
        self.action_history[1:, :] = self.action_history[:-1, :].copy()
        self.action_history[0, :] = actions.copy()

        # Zero out masked joints
        actions[self.mask_joint_idx] = 0.0

        # Publish target joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now_ros.to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = (actions * self.power_scale + self.init_pos).tolist()
        self.target_joint_states_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WalkPolicyInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
