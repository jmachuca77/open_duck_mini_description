cmake_minimum_required(VERSION 3.5)
project(open_duck_mini_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install URDF and meshes under share/<pkg>
install(DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files under share/<pkg>/launch
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

# Install your Python joint_state_publisher node
install(PROGRAMS
  src/mini_bdx_joint_state_publisher.py
  src/feet_switch_node.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FeetState.msg"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
)

ament_package()
