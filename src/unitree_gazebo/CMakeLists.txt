cmake_minimum_required(VERSION 3.5)
project(unitree_gazebo)

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
)

link_directories(${ament_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare Gazebo plugins as ROS 2 libraries
add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(unitreeFootContactPlugin ${ament_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(unitreeDrawForcePlugin ${ament_LIBRARIES} ${GAZEBO_LIBRARIES})

# Install Gazebo plugins
install(
    TARGETS
    unitreeFootContactPlugin
    unitreeDrawForcePlugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
)

# Install other resources or configuration files if needed
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Export the package for ROS 2
ament_export_include_directories(include)
ament_export_libraries(unitreeFootContactPlugin unitreeDrawForcePlugin)

ament_package()
