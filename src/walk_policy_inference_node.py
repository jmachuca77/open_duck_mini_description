#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import onnxruntime as ort
import numpy as np
import time
import os

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from open_duck_mini_description.msg import FeetState

from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.poly_reference_motion import PolyReferenceMotion


class WalkPolicyInferenceNode(Node):
    """
    ROS2 node that uses HWI solely to load DuckConfig (initial positions, joint names),
    then subscribes to /joint_states, /bno055/imu_raw, /feet_switch, /cmd_vel,
    builds the same 1×101 observation vector as RLWalk.get_obs(), runs the ONNX policy,
    masks out head/antenna actions, merges any head commands, publishes 14-joint targets on /target_joint_states,
    and publishes the current phase_frequency_factor on /phase_frequency_factor.
    """

    def __init__(self):
        super().__init__('walk_policy_inference_node')

        # --- 1. Load ONNX model with updated path and execution provider ---
        pkg_share = get_package_share_directory('open_duck_mini_description')
        model_path = os.path.join(pkg_share, 'checkpoints', 'BEST_WALK_ONNX_2.onnx')
        self.model = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

        # --- 2. Instantiate HWI & DuckConfig to get joint ordering and init_pos ---
        duck_cfg = DuckConfig()  # loads default ~/duck_config.json if present
        self.hwi = HWI(duck_cfg)
        # No need to call hwi.turn_on() here; we’re not reading hardware positions directly.

        # 2.1. Get the 14 hardware joints from HWI (dictionary keys are ordered)
        hw_joint_names = list(self.hwi.joints.keys())  # e.g. ["left_hip_yaw", "left_hip_roll", ...] (14 names)

        # 2.2. We need to insert two dummy “antenna” joints at index 9, just like JointStatePublisher does.
        self.dummy_joints = ["left_antenna", "right_antenna"]
        self.dummy_insert_idx = 9
        joint_names_16 = hw_joint_names.copy()
        joint_names_16[self.dummy_insert_idx:self.dummy_insert_idx] = self.dummy_joints
        # Now joint_names_16 is length 16.

        self.joint_names_16 = joint_names_16

        # 2.3. Identify antenna indices and DOF indices
        self.antenna_indices = [
            self.joint_names_16.index("left_antenna"),
            self.joint_names_16.index("right_antenna")
        ]
        # Indices of actual 14 DOFs = all except antenna indices
        self.dof_indices = [i for i in range(len(self.joint_names_16)) if i not in self.antenna_indices]

        # 2.4. Build init_pos16: for each name in joint_names_16, take from hwi.init_pos if present, else 0.0
        init_pos16 = []
        for name in self.joint_names_16:
            if name in self.hwi.init_pos:
                init_pos16.append(self.hwi.init_pos[name])
            else:
                init_pos16.append(0.0)  # dummy joint init = 0
        self.init_pos16 = np.array(init_pos16, dtype=np.float32)

        # 2.5. Extract 14-length init_pos (drop antenna entries)
        self.init_pos = self.init_pos16[self.dof_indices]

        # --- 3. Policy-related parameters ---

        # 3.1. Joint-velocity scale (policy uses dof_vel * 0.05)
        self.joint_vel_scale = 0.05

        # 3.2. Controller command vector “cmds” (length 7)
        # [forward_cmd, lateral_cmd, yaw_cmd, head_pan, head_tilt, head_roll, head_yaw]
        self.cmds = np.zeros(7, dtype=np.float32)

        # 3.3. Action-history arrays (each length 14)
        self.last_action = np.zeros(14, dtype=np.float32)
        self.last_last_action = np.zeros(14, dtype=np.float32)
        self.last_last_last_action = np.zeros(14, dtype=np.float32)

        # 3.4. Current motor_targets (14 floats), initialize to init_pos
        self.motor_targets = self.init_pos.copy()

        # 3.5. Feet-contacts placeholder (2 floats)
        self.feet_contacts = None

        # 3.6. Imitation-phase logic
        prm_path = os.path.join(get_package_share_directory('open_duck_mini_description'),
                                 'config/polynomial_coefficients.pkl')
        self.PRM = PolyReferenceMotion(prm_path)
        self.imitation_i = 0.0
        self.imitation_phase = np.zeros(2, dtype=np.float32)
        self.phase_frequency_factor = 1.0
        self.phase_frequency_factor_offset = 0.0

        # 3.7. Action scale (policy’s “action_scale” = 0.25)
        self.action_scale = 0.25

        # 3.8. Mask indices for joints: RLWalk masked out [
        #    'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'left_antenna', 'right_antenna'
        # ]
        mask_joints = ['neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
                       'left_antenna', 'right_antenna']
        self.mask_joint_idx = np.array([self.joint_names_16.index(j) for j in mask_joints], dtype=np.int32)

        # Note: those indices are in the 16-element space; when action is length 14,
        # we need to map them to the 14 DOF indices. But RLWalk’s ONNX expects a 14-length action vector
        # already zeroed out at those positions. In the RLWalk training, they used a 16-length joint list
        # but zeroed mask indices before computing motor_targets. Here the ONNX expects exactly length 14,
        # so mask_joint_idx must be converted to indices within the 14-vector space.
        # Actually, RLWalk’s action vector is length 14 (since it ignores antenna joints entirely).
        # Therefore, mask_joint_idx should be indices within [0..13] corresponding to the 14 dof_indices.
        # Let’s compute: for each joint in mask_joints that is in dof_indices, get its index within dof_indices.

        dof_mask_idx = []
        for joint in mask_joints:
            full_idx = self.joint_names_16.index(joint)
            if full_idx in self.dof_indices:
                dof_mask_idx.append(self.dof_indices.index(full_idx))
        self.dof_mask_idx = np.array(dof_mask_idx, dtype=np.int32)  # indices in [0..13]

        # --- 4. Placeholders & timestamps ---

        # Raw IMU data (each length 3)
        self.raw_gyro = None
        self.raw_accelero = None

        # Joint-state placeholders: from /joint_states topic (16 floats each)
        self.joint_positions_16 = None
        self.joint_velocities_16 = None

        # Timestamps for freshness (required at startup only)
        self.last_imu_update = None
        self.last_cmd_vel_update = None
        self.last_feet_contact_update = None
        self.last_joint_states_update = None

        # For logging missing inputs
        self.last_rate_log_time = time.time()

        # --- 5. Publishers & Subscribers ---

        self.target_joint_states_pub = self.create_publisher(
            JointState, '/target_joint_states', 1
        )

        self.imu_sub = self.create_subscription(
            Imu, '/bno055/imu_raw', self.imu_callback, 1
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1
        )

        self.feet_contact_sub = self.create_subscription(
            FeetState, '/feet_switch', self.feet_contact_callback, 1
        )

        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 1
        )

        # Publisher for phase_frequency_factor (debug)
        self.freq_pub = self.create_publisher(
            Float32, '/phase_frequency_factor', 1
        )

        # --- 6. save_obs parameter (defaults to False) ---
        self.declare_parameter('save_obs', False)
        self.save_obs = self.get_parameter('save_obs').get_parameter_value().bool_value
        self.saved_obs = [] if self.save_obs else None

        self.get_logger().info("Walk Policy Inference Node initialized.")

        # --- 7. Create 50 Hz timer to run inference ---
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.started = False

    def imu_callback(self, msg: Imu):
        """Store raw IMU gyro (rad/s) and accelero (m/s²)."""
        self.raw_gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)
        self.raw_accelero = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float32)
        self.last_imu_update = self.get_clock().now()

    def cmd_vel_callback(self, msg: Twist):
        """Fill cmds[0:3] = (linear.x, linear.y, angular.z). Then update imitation phase speed."""
        self.cmds[0] = msg.linear.x
        self.cmds[1] = msg.linear.y
        self.cmds[2] = msg.angular.z
        # cmds[3:7] remain as head commands (set externally if desired)

        # --- Update phase_frequency_factor exactly like RLWalk.get_phase_frequency_factor() ---
        x_vel = self.cmds[0]
        if abs(x_vel) < 1e-3:
            # If command is (essentially) zero, freeze phase (no walking)
            self.phase_frequency_factor = 0.0
        else:
            max_phase_frequency = 1.2
            min_phase_frequency = 1.0
            freq = min_phase_frequency + (abs(x_vel) / 0.15) * (
                max_phase_frequency - min_phase_frequency
            )
            # Clamp between [min_phase_frequency, max_phase_frequency]
            self.phase_frequency_factor = float(np.clip(freq, min_phase_frequency, max_phase_frequency))

        # Publish current frequency factor for debugging
        self.freq_pub.publish(Float32(data=self.phase_frequency_factor))

        self.last_cmd_vel_update = self.get_clock().now()

    def feet_contact_callback(self, msg: FeetState):
        """Extract left/right foot switch into a 2-float array."""
        left = float(msg.left_contact)
        right = float(msg.right_contact)
        self.feet_contacts = np.array([left, right], dtype=np.float32)
        self.last_feet_contact_update = self.get_clock().now()

    def joint_states_callback(self, msg: JointState):
        """
        Store the 16-element joint_positions_16 and joint_velocities_16 in the same order
        as JointStatePublisher published them.
        """
        # Assume msg.name matches self.joint_names_16 exactly, and msg.position/velocity length =16
        self.joint_positions_16 = np.array(msg.position, dtype=np.float32)
        self.joint_velocities_16 = np.array(msg.velocity, dtype=np.float32)
        self.last_joint_states_update = self.get_clock().now()

    def timer_callback(self):
        """
        Main loop at 50 Hz:
        - During startup, wait until raw_gyro, raw_accelero, cmds, feet_contacts, joint_states have been seen.
        - Once started, each iteration:
          • Read fresh joint_states (we have them from callback)
          • Compute dof_pos_minus_init (14 floats) and dof_vel_scaled (14 floats)
          • Update imitation_phase (2 floats)
          • Build obs (101 floats) in RLWalk order (and optionally save it)
          • Run ONNX inference, apply mask, update action history
          • Compute motor_targets (14 floats), then merge head commands
          • Publish /target_joint_states with those 14 joints
        """
        # --- 7.1. Startup check ---
        if not self.started:
            missing = []
            if (self.raw_gyro is None) or (self.raw_accelero is None):
                missing.append('IMU')
            if self.last_cmd_vel_update is None:
                missing.append('cmd_vel')
            if self.feet_contacts is None:
                missing.append('feet_contacts')
            if (self.joint_positions_16 is None) or (self.joint_velocities_16 is None):
                missing.append('joint_states')
            now = time.time()
            if missing:
                if now - self.last_rate_log_time > 1.0:
                    self.get_logger().info(f"Waiting for inputs: {missing}")
                    self.last_rate_log_time = now
                return

            # All inputs seen at least once → compute initial dof arrays
            dof_pos14 = self.joint_positions_16[self.dof_indices]
            dof_vel14 = self.joint_velocities_16[self.dof_indices]
            self.dof_pos_minus_init = dof_pos14 - self.init_pos
            self.dof_vel_scaled = dof_vel14 * self.joint_vel_scale

            self.get_logger().info("All sensor data received. Starting policy execution.")
            self.started = True
            return

        # --- 7.2. After startup: compute fresh DOF pos/vel from joint_states buffer ---
        dof_pos14 = self.joint_positions_16[self.dof_indices]
        dof_vel14 = self.joint_velocities_16[self.dof_indices]
        self.dof_pos_minus_init = dof_pos14 - self.init_pos
        self.dof_vel_scaled = dof_vel14 * self.joint_vel_scale

        # --- 7.3. Update imitation phase (2 floats) ---
        self.imitation_i = (
            self.imitation_i
            + 1 * (self.phase_frequency_factor + self.phase_frequency_factor_offset)
        ) % self.PRM.nb_steps_in_period
        self.imitation_phase = np.array([
            np.cos(self.imitation_i / self.PRM.nb_steps_in_period * 2 * np.pi),
            np.sin(self.imitation_i / self.PRM.nb_steps_in_period * 2 * np.pi),
        ], dtype=np.float32)

        # --- 7.4. Build 1×101 observation vector in exact RLWalk order ---
        obs = np.concatenate([
            self.raw_gyro,                #  3
            self.raw_accelero,            #  3
            self.cmds,                    #  7
            self.dof_pos_minus_init,      # 14
            self.dof_vel_scaled,          # 14
            self.last_action,             # 14
            self.last_last_action,        # 14
            self.last_last_last_action,   # 14
            self.motor_targets,           # 14
            self.feet_contacts,           #  2
            self.imitation_phase          #  2
        ], dtype=np.float32)               # total = 101

        # --- 7.5. Optionally save this observation ---
        if self.save_obs:
            self.saved_obs.append(obs.copy())

        # --- 7.6. Run ONNX inference ---
        output = self.model.run(None, {'obs': obs.reshape(1, -1)})
        action = output[0].flatten()  # length 14

        # --- 7.7. Mask out head/antenna indices (within 14-vector) ---
        action[self.dof_mask_idx] = 0.0

        # --- 7.8. Update action history ---
        self.last_last_last_action = self.last_last_action.copy()
        self.last_last_action = self.last_action.copy()
        self.last_action = action.copy()

        # --- 7.9. Compute new motor_targets (14 floats) ---
        self.motor_targets = self.init_pos + action * self.action_scale

        # --- Merge head commands exactly like RLWalk ---
        # RLWalk did: head_motor_targets = last_commands[3:] + motor_targets[5:9]
        # and then: motor_targets[5:9] = head_motor_targets
        head_cmds = self.cmds[3:7]  # [head_pan, head_tilt, head_roll, head_yaw]
        self.motor_targets[5:9] = head_cmds + self.motor_targets[5:9]

        # --- 7.10. Publish the 14-joint targets on /target_joint_states ---
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        published_names = [self.joint_names_16[i] for i in self.dof_indices]
        msg.name = published_names
        msg.position = self.motor_targets.tolist()
        self.target_joint_states_pub.publish(msg)

    def destroy_node(self):
        # No hardware to turn off here; JointStatePublisher handles HWI commands.
        super().destroy_node()
        self.get_logger().info('Shutting down walk_policy_inference_node')

        # --- 8. If save_obs was enabled, dump saved_obs to disk ---
        if self.save_obs and (self.saved_obs is not None):
            try:
                import pickle
                with open('ros_saved_obs.pkl', 'wb') as f:
                    pickle.dump(self.saved_obs, f)
                self.get_logger().info(f"Saved {len(self.saved_obs)} observations to ros_saved_obs.pkl")
            except Exception as e:
                self.get_logger().error(f"Failed to save observations: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WalkPolicyInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
