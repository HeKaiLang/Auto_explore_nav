cmake_minimum_required(VERSION 3.0.2)

project(mine_map_deploy)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    geometry_msgs
    move_base_flex_msgs
    roscpp
    sensor_msgs
    nav_msgs
    std_srvs
    tf
    visualization_msgs
    pcl_conversions
    cv_bridge
    image_transport
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(image_publisher
    src/image_publisher.cpp)

target_link_libraries(image_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable(global_map_listener
    src/global_map_listen.cpp)

add_executable(control_to_cmdvel_node src/control_to_cmdvel_node.cpp)
target_link_libraries(control_to_cmdvel_node ${catkin_LIBRARIES})
add_dependencies(control_to_cmdvel_node ${catkin_EXPORTED_TARGETS})

add_executable(cmdvel_to_control_node src/cmdvel_to_control_node.cpp)
target_link_libraries(cmdvel_to_control_node ${catkin_LIBRARIES})
add_dependencies(cmdvel_to_control_node ${catkin_EXPORTED_TARGETS})

add_executable(udp_control_bridge src/udp_control_bridge.cpp)
target_link_libraries(udp_control_bridge ${catkin_LIBRARIES})

add_executable(control_to_cmdvel_node src/control_to_cmdvel_node.cpp)
target_link_libraries(control_to_cmdvel_node ${catkin_LIBRARIES})

add_executable(cmdvel_to_serial_node src/cmdvel_to_serial_node.cpp)
target_link_libraries(cmdvel_to_serial_node ${catkin_LIBRARIES})

target_link_libraries(global_map_listener
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable(selected_marker_nav
    src/selected_marker_nav.cpp)

target_link_libraries(selected_marker_nav
    ${catkin_LIBRARIES})
