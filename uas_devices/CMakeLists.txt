cmake_minimum_required(VERSION 3.8)
project(uas_devices)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_px4 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# YDLidar Devices
find_package(ydlidar_sdk REQUIRED)  # Needs to be installed before

add_library(uas_devices SHARED
    "src/ydlidar/ydlidar_node.cpp"
)
target_link_libraries(uas_devices
    ${YDLIDAR_SDK_LIBRARIES}
)
ament_target_dependencies(uas_devices
    rclcpp
    rclcpp_components
    sensor_msgs
    std_srvs
)
rclcpp_components_register_nodes(uas_devices
    "uas_devices::YDLidarNode"
)

add_library(behavior_tree_nodes SHARED
    "src/behavior_tree/lidar/detect_obstacle.cpp"
)
ament_target_dependencies(behavior_tree_nodes
    auto_apms_behavior_tree
    sensor_msgs
)

auto_apms_behavior_tree_declare_nodes(behavior_tree_nodes 
    "uas_devices::bt::DetectObstacle"
    NODE_MANIFEST
    "config/lidar_node_manifest.yaml"
)

auto_apms_behavior_tree_declare_trees( 
    "config/tree/lidar.xml"
    NODE_MANIFEST
    "auto_apms_px4::behavior_tree_nodes"
    "uas_devices::behavior_tree_nodes"
)

# Executables
# install(
#     TARGETS
    
#     DESTINATION "lib/${PROJECT_NAME}"
# )

# Libraries
install(
    TARGETS
    uas_devices
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
  FILES "config/ydlidar_tmini_pro.rviz"
  DESTINATION DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
