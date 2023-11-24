import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    projection_source_root_frame_arg = DeclareLaunchArgument(
        'projection_source_root_frame',
    )
    projection_source_frame_arg = DeclareLaunchArgument(
        'projection_source_frame',
    )
    projection_target_root_frame_arg = DeclareLaunchArgument(
        'projection_target_root_frame',
    )
    projection_target_frame_arg = DeclareLaunchArgument(
        'projection_target_frame',
    )
    projection_target_attachment_frame_arg = DeclareLaunchArgument(
        'projection_target_attachment_frame',
    )

    return LaunchDescription([
        projection_source_root_frame_arg,
        projection_source_frame_arg,
        projection_target_root_frame_arg,
        projection_target_frame_arg,
        projection_target_attachment_frame_arg,
        Node(
            package='tf2_projector',
            executable='tf2_projector',
            name='tf2_projector',
            parameters=[{
                'projection_source_root_frame': ParameterValue(LaunchConfiguration('projection_source_root_frame'), value_type=str),
                'projection_source_frame': ParameterValue(LaunchConfiguration('projection_source_frame'), value_type=str),
                'projection_target_root_frame': ParameterValue(LaunchConfiguration('projection_target_root_frame'), value_type=str),
                'projection_target_frame': ParameterValue(LaunchConfiguration('projection_target_frame'), value_type=str),
                'projection_target_attachment_frame': ParameterValue(LaunchConfiguration('projection_target_attachment_frame'), value_type=str),
            }]
        )
    ])
