"""Read-only shared-camera consumer; never starts camera/arm/legacy executor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('weights'), DeclareLaunchArgument('depth_scale_16u'),
        Node(package='perception_node', executable='perception_node',
             parameters=[{'weights': LaunchConfiguration('weights'),
                          'depth_scale_16u': ParameterValue(LaunchConfiguration('depth_scale_16u'), value_type=float),
                          'integrated_target_required': True,
                          'rgb_topic':'/shared_camera/color/image_raw',
                          'depth_topic':'/shared_camera/aligned_depth_to_color/image_raw',
                          'camera_info_topic':'/shared_camera/color/camera_info'}])])
