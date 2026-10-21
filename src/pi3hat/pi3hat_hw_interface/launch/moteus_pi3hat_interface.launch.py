import os
import subprocess
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

BAG_TOPICS = [
    '/events/write_split',
    '/imu_sensor/imu',
    '/ik_controller/base_pose',
    '/joy',
    '/joy/set_feedback',
    '/nuc_heartbeat',
    '/omni_controller/direct_wheels_cmd',
    '/omni_controller/distributors_state',
    '/omni_controller/debug/joints_command',
    '/omni_controller/joints_state',
    '/omni_controller/joints_reference',
    '/omni_controller/performance',
    '/omni_controller/safety_state',
    '/omni_controller/transition_event',
    '/omni_controller/twist_cmd',
    '/omni_mulinex_joystick/identification/segment',
    '/state_broadcaster/joints_state',
    '/state_broadcaster/performance_indexes',
    '/state_broadcaster/transition_event',
]


def _save_git_diff(repo_dir: str, out_file: str) -> None:
    try:
        head = subprocess.check_output(
            ['git', '-C', repo_dir, 'log', '-1', '--oneline'],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        diff = subprocess.check_output(
            ['git', '-C', repo_dir, 'diff', 'HEAD'],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        with open(out_file, 'w') as f:
            f.write(f'# HEAD: {head}')
            f.write(diff)
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        with open(out_file, 'w') as f:
            f.write(f'# git info unavailable: {e}\n')


def _setup_recording(context, *args, **kwargs):
    if LaunchConfiguration('record_bag').perform(context).lower() not in ('true', '1', 'yes'):
        return []

    # Workspace root is 4 levels up from <ws>/install/<pkg>/share/<pkg>
    workspace_dir = os.path.abspath(
        os.path.join(get_package_share_directory('pi3hat_hw_interface'), '..', '..', '..', '..')
    )
    run_dir = os.path.join(workspace_dir, 'bags', datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    os.makedirs(run_dir, exist_ok=True)
    _save_git_diff(workspace_dir, os.path.join(run_dir, 'git_diff.txt'))

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', *BAG_TOPICS, '-o', os.path.join(run_dir, 'bag')],
            output='screen',
        )
    ]


def generate_launch_description():
    urdf_name_arg = DeclareLaunchArgument('urdf_file', default_value='omnicar.xacro')
    conf_name_arg = DeclareLaunchArgument('conf_file', default_value='omnicar.yaml')
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
    )

    robot_model_path = PathJoinSubstitution(
        [FindPackageShare('pi3hat_hw_interface'), 'urdf', LaunchConfiguration('urdf_file')]
    )
    robot_description = ParameterValue(Command(['xacro ', robot_model_path]), value_type=str)

    controller_path = PathJoinSubstitution(
        [FindPackageShare('pi3hat_hw_interface'), 'config', LaunchConfiguration('conf_file')]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_path],
    )

    state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['state_broadcaster'],
    )

    omni_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_controller'],
    )

    imu_sensor_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor'],
    )

    return LaunchDescription(
        [
            urdf_name_arg,
            conf_name_arg,
            record_bag_arg,
            control_node,
            state_broadcaster_spawner,
            omni_controller_spawner,
            imu_sensor_spawner,
            OpaqueFunction(function=_setup_recording),
        ]
    )
