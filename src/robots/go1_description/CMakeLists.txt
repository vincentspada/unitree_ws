cmake_minimum_required(VERSION 3.5)
project(go1_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${ament_INCLUDE_DIRS}
)

# Link directories (if necessary)
link_directories(${ament_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIR})
link_directories(${rclcpp_LIBRARY_DIR})
link_directories(${std_msgs_LIBRARY_DIR})
link_directories(${tf2_LIBRARY_DIR})

#add_executable(go1_description_node src/go1_description_node.cpp)
#go1_description_node
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    tf2
)
#(go1_description_node ${PROJECT_NAME})
#go1_description_node
install(TARGETS
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()