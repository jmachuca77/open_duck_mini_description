# CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(open_duck_mini_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install URDF and meshes under share/<pkg>
install(DIRECTORY urdf meshes config checkpoints
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files under share/<pkg>/launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes
install(PROGRAMS
  src/mini_bdx_joint_state_publisher.py
  src/feet_switch_node.py
  src/walk_policy_inference_node.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FeetState.msg"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
