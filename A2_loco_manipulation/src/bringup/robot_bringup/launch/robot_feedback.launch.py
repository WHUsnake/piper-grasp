"""Publish robot TF from measured joints only; no drivers or commands."""
from pathlib import Path
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    model = Path(LaunchConfiguration('model').perform(context))
    description = model.read_text(encoding='utf-8')
    calibration = yaml.safe_load(Path(LaunchConfiguration('handeye').perform(context)).read_text())['end_from_camera_link']
    xyz = calibration['translation_m']
    quat = calibration['quaternion_xyzw']
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='piper_feedback_state_publisher', output='screen',
        parameters=[{'robot_description': ParameterValue(description, value_type=str),
                     'publish_frequency': 30.0}],
        remappings=[('joint_states', '/joint_states')],
    ), Node(
        package='robot_bringup', executable='joint_state_relay',
        name='piper_joint_state_relay', output='screen',
    ), Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='piper_world_base_static_tf', output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'world', '--child-frame-id', 'base_link'],
    ), Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='piper_handeye_static_tf', output='screen',
        # The measured hand-eye transform was link6 -> color optical frame.
        # Compose it with the Orbbec's fixed camera_link -> color optical TF,
        # so the optical frame keeps exactly one parent in the TF tree.
        arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                   '--qx', str(quat[0]), '--qy', str(quat[1]), '--qz', str(quat[2]), '--qw', str(quat[3]),
                   '--frame-id', 'link6', '--child-frame-id', 'camera_link'],
    )]


def generate_launch_description():
    root = os.environ['A2_PROJECT_ROOT']
    return LaunchDescription([
        DeclareLaunchArgument('handeye', default_value=root+'/config/calibration/handeye.yaml'),
        DeclareLaunchArgument('model', default_value=
            PathJoinSubstitution([FindPackageShare('piper_description'), 'urdf', 'piper_description.urdf'])),
        OpaqueFunction(function=start),
    ])
