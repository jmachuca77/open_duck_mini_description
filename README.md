# open_duck_mini_description

Robot description and state‐publisher package for the mini_bdx platform.  
This single package contains your URDF, meshes, launch files, custom publishers, a custom message, and a walk policy inference node.

## Package Structure

```text
open_duck_mini_description/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── FeetState.msg
├── urdf/
│   └── mini_bdx.urdf
├── meshes/            ← put your existing mesh files here
├── launch/
│   └── display.launch.py
└── src/
    ├── mini_bdx_joint_state_publisher.py
    ├── feet_switch_node.py
    └── walk_policy_inference_node.py
```

## Contents

- **URDF & meshes**  
  - `urdf/mini_bdx.urdf`  
  - `meshes/` (external files; not included here)

- **Launch**  
  - `display.launch.py`  
    - Loads your URDF into `/robot_description`  
    - Runs either the built-in `joint_state_publisher` or your hardware‐based `mini_bdx_joint_state_publisher`  
    - Optionally launches: GUI slider, RViz 2, Foxglove bridge  
    - Always launches:
      - `robot_state_publisher`  
      - `bno055` IMU driver (I²C)  
      - `feet_switch_node` (publishes foot‐contact status)  
      - `walk_policy_inference_node` (publishes `/target_joint_states`)

- **Custom message**  
  - `msg/FeetState.msg`  
    ```plain
    std_msgs/Header header
    bool left_contact
    bool right_contact
    ```

- **Python nodes**  
  - `mini_bdx_joint_state_publisher.py`  
    - Polls `HWI` for joint positions & velocities  
    - Publishes to `/joint_states`  
  - `feet_switch_node.py`  
    - Uses Adafruit Blinka `board` & `digitalio` to read two foot‐switch pins  
    - Publishes a `FeetState` message at 50 Hz on `/feet_switch`  
  - `walk_policy_inference_node.py`  
    - Loads an ONNX model (`policy_low_vel2.onnx` in `assets/`)  
    - Subscribes to:
      - `/cmd_vel` (`geometry_msgs/Twist`)  
      - `/feet_switch` (`std_msgs/Int32`)  
      - `/current_joint_states` (`sensor_msgs/JointState`)  
      - `/imu/data` (`sensor_msgs/Imu`)  
    - Publishes `/target_joint_states` (`sensor_msgs/JointState`) at 50 Hz

## Dependencies

Make sure you have these installed (for Jazzy on Ubuntu 22.04):

```bash
sudo apt update
sudo apt install \
  python3-blinka \
  ros-jazzy-bno055 \
  ros-jazzy-rviz2 \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-urdf \
  build-essential \
  python3-empy \
  python3-catkin-pkg-modules \
  python3-onnxruntime \
  python3-numpy \
  python3-pip
```

Also ensure your Python venv (if used) has:

```bash
pip install empy lark-parser PyYAML onnxruntime numpy
```

## Building

From your workspace root:

```bash
colcon build --packages-select open_duck_mini_description
source install/setup.bash
```

## Launch & Run

```bash
ros2 launch open_duck_mini_description display.launch.py
```

### Launch arguments

- `use_hw_jsp` (bool, default=`true`)  
  Use the hardware-based joint_state_publisher (`true`) or the built-in one (`false`).

- `enable_rviz` (bool, default=`false`)  
  Launch RViz 2.

- `enable_jsp_gui` (bool, default=`false`)  
  Launch the joint‐state slider panel.

- `enable_foxglove_bridge` (bool, default=`true`)  
  Launch the Foxglove WebSocket bridge.

#### Example: RViz + GUI, built-in JSP

```bash
ros2 launch open_duck_mini_description display.launch.py \
  use_hw_jsp:=false \
  enable_rviz:=true \
  enable_jsp_gui:=true
```

## Topics & Messages

- **`/joint_states`** (`sensor_msgs/msg/JointState`)  
  From your HWI hardware node or built-in publisher.

- **`/feet_switch`** (`open_duck_mini_description/msg/FeetState`)  
  Custom message containing header + two booleans (`left_contact`, `right_contact`).

- **`/target_joint_states`** (`sensor_msgs/msg/JointState`)  
  From `walk_policy_inference_node`.

- **IMU topics** from `bno055` driver:
  - `/imu/data` (`sensor_msgs/msg/Imu`)
  - `/mag` (`sensor_msgs/msg/MagneticField`)
  - `/temp` (`sensor_msgs/msg/Temperature`)

## Notes

- Ensure `/dev/i2c-1` and `/dev/ttyACM0` are accessible inside your container or host.  
- Place `assets/policy_low_vel2.onnx` under `open_duck_mini_description/policies/`.  
- Clean up GPIO pins on shutdown—nodes handle this automatically.

Happy building and testing your mini_bdx robot!
