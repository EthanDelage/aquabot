from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.event_handlers import OnProcessExit, OnShutdown
import os


def generate_launch_description():
    ld = LaunchDescription()

    benchmark_node = Node(
        package='benchmark',
        executable='benchmark',
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
            'world': 'aquabot_benchmark',
        }.items(),
    )
    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=benchmark_node,
            on_exit=[
                aquabot_launch_file,
            ]
        )
    )

    ld.add_action(benchmark_node)
    ld.add_action(navigation_node)
    ld.add_action(pathfinding_node)
    # ld.add_action(aquabot_launch_file)
    ld.add_action(event_handler)

    return ld
