from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown


def generate_launch_description():
    ld = LaunchDescription()

    # Ros Nodes launched
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

    # Aquabot Launch File
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
            'headless': 'false'
        }.items(),
    )
    ExecuteProcess(
        cmd=['python3', 'src/benchmark/resource/error_log'],
        output='screen',
    )
    # Event triggered when benchmark scenario is created to launch simulation
    benchmark_exit_event = RegisterEventHandler(
        OnProcessExit(
            target_action=benchmark_node,
            on_exit=[
                aquabot_launch_file,
                navigation_node,
                pathfinding_node,
                task_info_node,
            ]
        )
    )
    benchmark_exit_event = RegisterEventHandler(
        OnProcessExit(
            target_action=aquabot_launch_file,
            on_exit=[

            ]
        )
    )
    # Event triggered when task_info_node exit to shutdown the simulation
    task_success_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=task_info_node,
            on_exit=[
                LogInfo(msg='Task success'),
                EmitEvent(
                    event=Shutdown()
                )
            ]
        )
    )

    # Creating new scenario and launching simulation
    # ld.add_action(benchmark_node)
    # ld.add_action(benchmark_exit_event)

    # Launching simulation
    ld.add_action(aquabot_launch_file)

    # Launching Ros Nodes
    ld.add_action(task_info_node)
    ld.add_action(navigation_node)
    ld.add_action(pathfinding_node)
    # Event triggered to shutdown the simulation
    ld.add_action(task_success_handler)

    return ld
