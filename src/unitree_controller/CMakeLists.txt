cmake_minimum_required(VERSION 3.5)
project(unitree_controller)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(genmsg REQUIRED)
find_package(joint_state_controller REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)

# Declare ROS 2 package dependencies
ament_export_dependencies(unitree_legged_msgs)

# Include directories
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${ament_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${controller_interface_INCLUDE_DIRS}
    ${hardware_interface_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${unitree_legged_msgs_INCLUDE_DIRS}
)

# Link directories (if necessary)
link_directories(${ament_LIBRARY_DIRS})

link_directories(${Boost_LIBRARY_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${controller_interface_LIBRARY_DIRS})
link_directories(${hardware_interface_LIBRARY_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS})
link_directories(${realtime_tools_LIBRARY_DIRS})
link_directories(${unitree_legged_msgs_LIBRARY_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp
)

# Add dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  joint_state_controller
  robot_state_publisher
  gazebo_ros
  geometry_msgs
  unitree_legged_msgs
)

# Add dependencies for the library
target_link_libraries(${PROJECT_NAME}
  ${ament_LIBRARIES}
)

# Declare ROS 2 executables
add_executable(unitree_external_force src/external_force.cpp)
ament_target_dependencies(unitree_external_force
  rclcpp
  std_msgs
  unitree_legged_msgs
)
target_link_libraries(unitree_external_force ${PROJECT_NAME})

add_executable(unitree_servo src/servo.cpp)
ament_target_dependencies(unitree_servo
  rclcpp
  std_msgs
  unitree_legged_msgs
)
target_link_libraries(unitree_servo ${PROJECT_NAME})

add_executable(unitree_move_kinetic src/move_publisher.cpp)
ament_target_dependencies(unitree_move_kinetic
  rclcpp
  std_msgs
  unitree_legged_msgs
)
target_link_libraries(unitree_move_kinetic ${PROJECT_NAME})

# Install ROS 2 executables
install(TARGETS
  unitree_external_force
  unitree_servo
  unitree_move_kinetic
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
