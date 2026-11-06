import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def resolve_pipeline_paths():
    """Resolve deployment defaults without starting any ROS nodes or devices."""
    root = Path(os.environ["A2_PROJECT_ROOT"]).absolute()
    paths = {
        "python": Path(os.environ["A2_PYTHON_EXECUTABLE"]).absolute(),
        "weights": root / "data/models/yolo11s-seg.pt",
        "perception": root / "config/scenarios/pick_and_hold/perception.yaml",
        "moveit": root / "config/scenarios/pick_and_hold/moveit.yaml",
        "task": root / "config/scenarios/pick_and_hold/task.yaml",
    }
    for key, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(f"Pipeline {key} does not exist: {path}")
    if not os.access(paths["python"], os.X_OK):
        raise PermissionError(f"Pipeline Python is not executable: {paths['python']}")
    return {key: str(path) for key, path in paths.items()}


def require_preview(context):
    if LaunchConfiguration('grasp_backend').perform(context) == 'graspnet' and LaunchConfiguration('execute').perform(context).lower() != 'false':
        raise RuntimeError('GraspNet TCP geometry is unverified; execute must remain false')
    return []


def generate_launch_description():
    paths = resolve_pipeline_paths()
    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("grasp_backend", default_value="graspnet"),
        DeclareLaunchArgument("graspnet_config", default_value=os.path.join(get_package_share_directory("grasp_generation"), "config", "graspnet.json")),
        DeclareLaunchArgument('python',default_value=paths['python'],description='Project independent Python interpreter'),
        DeclareLaunchArgument('weights',default_value=paths['weights'],description='Absolute path of models/yolo11s-seg.pt'),
        DeclareLaunchArgument('device',default_value='cpu'),
        OpaqueFunction(function=require_preview),
        ExecuteProcess(cmd=[LaunchConfiguration('python'),'-c',
            'from perception_node.node import main; main()','--ros-args',
            '--params-file', paths['perception'],
            '-p',['weights:=',LaunchConfiguration('weights')],
            '-p',['device:=',LaunchConfiguration('device')]],output='screen'),
        Node(package='grasp_generation', executable='grasp_generator', output='screen',
             parameters=[{'backend': LaunchConfiguration('grasp_backend'),
                          'graspnet_config': LaunchConfiguration('graspnet_config')}]),
        Node(package='robot_planner', executable='robot_planner', output='screen',
             parameters=[paths['moveit']]),
        Node(package='task_executor', executable='task_executor', output='screen',
             parameters=[paths['task'], {'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool)}]),
    ])
