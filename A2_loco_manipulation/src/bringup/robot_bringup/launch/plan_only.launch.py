"""Real joint feedback + MoveIt planning server. Does not execute trajectories."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Use the same physical model as robot_feedback, without Gazebo plugins.
    model = Path(get_package_share_directory('piper_description')) / 'urdf/piper_description.urdf'
    config = (MoveItConfigsBuilder('piper', package_name='piper_with_gripper_moveit')
              .robot_description(file_path=str(model)).to_moveit_configs())
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_bringup'), 'launch', 'robot_feedback.launch.py']))),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[config.to_dict(), {'allow_trajectory_execution': False,
                                           'use_sim_time': False,
                                           'publish_robot_description_semantic': True}]),
    ])
