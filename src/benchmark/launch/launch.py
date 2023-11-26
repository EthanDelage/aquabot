from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
import os

def handle_node_exit(context, process_info, **kwargs):
    status_code = process_info['return_code']
    print(f'Status code: {status_code}')


def generate_launch_description():
    ld = LaunchDescription()

    benchmark_node = Node(
        package='benchmark',
        executable='benchmark',
    )
    task_info_node = Node(
        package='benchmark',
        executable='task_info',
    )
    navigation_node = Node(
        package='navigation',
        executable='navigation',
    )
    pathfinding_node = Node(
        package='pathfinding',
        executable='pathfinding',
    )
    aquabot_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aquabot_gz'),
                'launch',
                'competition.launch.py'
            ])
        ),
        launch_arguments={
            'world': 'aquabot_task_hard',
        }.items(),
    )
    launch_exit_event = RegisterEventHandler(
        OnProcessExit(
            target_action=benchmark_node,
            on_exit=[
                aquabot_launch_file,
                navigation_node,
                pathfinding_node,
            ]
        )
    )
    task_success_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=task_info_node,
            on_exit=[
                LogInfo(msg='Task success'),
                EmitEvent(
                    event=Shutdown(reason='Task success')
                )
            ]
        )
    )

    # ld.add_action(benchmark_node)
    # ld.add_action(launch_exit_event)
    
    ld.add_action(task_info_node)
    ld.add_action(navigation_node)
    ld.add_action(pathfinding_node)
    ld.add_action(aquabot_launch_file)
    ld.add_action(task_success_handler)

    return ld
