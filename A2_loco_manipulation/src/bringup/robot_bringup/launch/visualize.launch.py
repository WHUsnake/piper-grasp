"""View the running perception stream; starts no drivers or motion nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fixed_frame', default_value='camera_color_optical_frame'),
        Node(package='rviz2', executable='rviz2', name='bottle_cloud_viewer',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('robot_bringup'), 'config', 'bottle_cloud.yaml']),
                 '-f', LaunchConfiguration('fixed_frame')], output='screen'),
    ])
