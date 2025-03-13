import numpy as np

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    driver_node = Node(
        package="uas_ros2",
        executable="ydlidar_pub_node",
        name="ydlidar_pub_node",
        output="screen",
        emulate_tty=True,
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
    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_lidar"],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory("uas_ros2"), "ydlidar_tminipro.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            driver_node,
            # tf2_node,
            # rviz2_node
        ]
    )
