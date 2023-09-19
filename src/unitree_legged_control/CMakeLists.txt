cmake_minimum_required(VERSION 3.5)
project(unitree_legged_control)

# Find required ROS 2 packages and libraries
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(unitree_legged_msgs REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Declare the ROS 2 package
ament_package()

# Include directories
include_directories(
    include
    ${ament_INCLUDE_DIRS}
    ${controller_interface_INCLUDE_DIRS}
    ${hardware_interface_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${unitree_legged_msgs_INCLUDE_DIRS}
)

# Link directories (if necessary)
link_directories(${ament_LIBRARY_DIRS})
link_directories(${controller_interface_LIBRARY_DIRS})
link_directories(${hardware_interface_LIBRARY_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS})
link_directories(${realtime_tools_LIBRARY_DIRS})
link_directories(${unitree_legged_msgs_LIBRARY_DIRS})

# Create a ROS 2 library
add_library(unitree_legged_control
    src/joint_controller.cpp
)

# Add dependencies and link libraries
ament_target_dependencies(unitree_legged_control
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    realtime_tools
    unitree_legged_msgs
)

# Install the library
install(TARGETS unitree_legged_control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
