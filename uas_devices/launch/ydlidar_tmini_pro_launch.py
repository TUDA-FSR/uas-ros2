from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    container = ComposableNodeContainer(
        name="ydlidar_launch",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                name="ydlidar_tmini_pro",
                package="uas_devices",
                plugin="uas_devices::YDLidarNode",
                parameters=[
                    {
                        "port": "/dev/ttyUSB0",
                        "frame_id": "base_lidar",
                        "baudrate": 230400,
                        "lidar_type": 1,
                        "device_type": 0,
                        "sample_rate": 4,
                        "intensity_bit": 8,
                        "abnormal_check_count": 4,
                        "fixed_resolution": True,
                        "reversion": True,
                        "inverted": True,
                        "auto_reconnect": True,
                        "isSingleChannel": False,
                        "intensity": True,
                        "support_motor_dtr": False,
                        "angle_max": 180.0,
                        "angle_min": -180.0,
                        "range_max": 12.0,
                        "range_min": 0.03,
                        "frequency": 10.0,
                        "invalid_range_is_inf": False,
                    }
                ],
            )
        ],
    )

    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_lidar",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_lidar"],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory("uas_devices"), "ydlidar_tmini_pro.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            container,
            #   tf2_node,
            #   rviz2_node
        ]
    )
