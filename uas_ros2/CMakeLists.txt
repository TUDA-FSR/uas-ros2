cmake_minimum_required(VERSION 3.8)
project(uas_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_px4 REQUIRED)
find_package(laser_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ydlidar_sdk REQUIRED)
add_executable(ydlidar_pub_node
    "src/ydlidar_pub_node.cpp"
)
ament_target_dependencies(ydlidar_pub_node
    rclcpp
    sensor_msgs
    std_srvs
)
target_link_libraries(ydlidar_pub_node
    ${YDLIDAR_SDK_LIBRARIES}
)

add_library(behavior_tree_nodes SHARED
    "src/behavior_tree/lidar/detect_obstacle.cpp"
)
ament_target_dependencies(behavior_tree_nodes
    auto_apms_behavior_tree
    laser_filters
    sensor_msgs
)

auto_apms_behavior_tree_declare_nodes(behavior_tree_nodes 
    "uas_ros2::bt::DetectObstacle"
    NODE_MANIFEST
    "config/lidar_node_manifest.yaml"
)

auto_apms_behavior_tree_declare_trees( 
    "config/tree/lidar.xml"
    NODE_MANIFEST
    "auto_apms_px4::behavior_tree_nodes"
    "uas_ros2::behavior_tree_nodes"
)

install(
    TARGETS
    ydlidar_pub_node
    DESTINATION "lib/${PROJECT_NAME}"
)

install(
    TARGETS
    behavior_tree_nodes
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  DIRECTORY "launch"
  DESTINATION DESTINATION "share/${PROJECT_NAME}"
)

install(
  FILES "config/ydlidar_tminipro.rviz"
  DESTINATION DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
