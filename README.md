# open_duck_mini_description

Robot description and state‐publisher package for the mini_bdx platform.  
This “single‐package” contains your URDF, meshes, launch files, custom publishers, and a custom message type.

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
    └── feet_switch_node.py
```

## Contents

- **URDF & meshes**
  - `urdf/mini_bdx.urdf`  
  - `meshes/` (external files; not included here)

- **Launch**
  - `display.launch.py`
    - Loads your URDF into `/robot_description`
    - Runs **either** the built-in `joint_state_publisher` **or** your hardware-based `mini_bdx_joint_state_publisher`
    - Optionally launches the GUI slider, RViz 2, and Foxglove bridge
    - Always launches:
      - `robot_state_publisher`
      - `bno055` IMU driver (I²C)
      - `feet_switch_node` (publishes foot-contact status)

- **Custom message**
  - `msg/FeetState.msg`
  
```plain
std_msgs/Header header
bool left_contact
bool right_contact
```

- **Python nodes**
  - `mini_bdx_joint_state_publisher.py`
    - Polls your `HWI` class for joint positions & velocities
    - Publishes to `/joint_states`
  - `feet_switch_node.py`
    - Uses Adafruit Blinka `board` + `digitalio` to read two foot-switch pins
    - Publishes a `FeetState` message at 50 Hz on `/feet_switch`

## Dependencies

Make sure you have these installed (for **Jazzy** on Ubuntu 22.04):

```bash
sudo apt update
sudo apt install \
  python3-blinka \
  ros-jazzy-bno055 \
  ros-jazzy-rviz2 \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  build-essential \
  python3-empy \
  python3-catkin-pkg-modules
```

Also ensure your Python 3 virtual environment (if any) has:

```bash
pip install empy lark-parser PyYAML
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
  Launch the joint-state slider panel inside or alongside RViz.

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

- **IMU topics** from `bno055` driver:
  - `/imu` (`sensor_msgs/msg/Imu`)
  - `/mag` (`sensor_msgs/msg/MagneticField`)
  - `/temp` (`sensor_msgs/msg/Temperature`)

## Notes

- Ensure your I²C bus (`/dev/i2c-1`) and serial port (`/dev/ttyACM0`) are accessible inside your container or on your host.  
- Be sure to wire PS1 on the BNO055 for UART mode, or leave it in I²C mode for the default driver config.  
- Clean up your GPIO pins on shutdown—both nodes do this automatically.

Happy building and testing your mini_bdx robot!
