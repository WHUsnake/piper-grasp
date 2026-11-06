"""Single-view legacy pipeline with real feedback; no active observation."""
import os
import sys
import fcntl
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(package, filename, condition=None, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        condition=condition, launch_arguments=(arguments or {}).items())


def claim_single_instance(context):
    domain = re.sub(r'[^0-9]', '_', os.environ.get('ROS_DOMAIN_ID', '0'))
    path = f'/tmp/she-single-view-{os.getuid()}-{domain}.lock'
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        raise RuntimeError('single_view is already running in this ROS domain; do not start a second instance')
    # Keep the descriptor open for this launch process lifetime; exec children
    # do not inherit it. Process exit releases the lock, including on failure.
    return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=claim_single_instance),
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('driver_python', default_value=os.environ.get('A2_PYTHON_EXECUTABLE', sys.executable)),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_image', default_value='true'),
        Node(package='piper', executable='piper_single_ctrl',
             prefix=[LaunchConfiguration('driver_python')],
             name='piper_ctrl_single_node', output='screen',
             parameters=[{'can_port': LaunchConfiguration('can_port'),
                          'auto_enable': ParameterValue(LaunchConfiguration('auto_enable'), value_type=bool),
                          'gripper_exist': True, 'cartesian_speed_percent': 10}],
             condition=IfCondition(LaunchConfiguration('start_driver'))),
        include('piper_vision_grasp', 'orbbec.launch.py',
                IfCondition(LaunchConfiguration('start_camera'))),
        include('robot_bringup', 'plan_only.launch.py',
                IfCondition(LaunchConfiguration('start_moveit'))),
        include('robot_bringup', 'pipeline.launch.py',
                arguments={'execute': LaunchConfiguration('execute')}),
        include('robot_bringup', 'visualize.launch.py',
                IfCondition(LaunchConfiguration('start_rviz')),
                {'fixed_frame': 'base_link'}),
        Node(package='rqt_image_view', executable='rqt_image_view',
             name='she_color_image_view', arguments=['/camera/color/image_raw'],
             output='screen', condition=IfCondition(LaunchConfiguration('start_image'))),
    ])
