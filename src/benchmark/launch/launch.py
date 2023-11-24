from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
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
            os.path.join(get_package_share_directory('aquabot_gz'), 'launch/competition.launch.py')
        ),
        launch_arguments={'world': 'aquabot_task_medium'}.items(),
    )

    # ld.add_action(benchmark_node)
    # ld.add_action(navigation_node)
    # ld.add_action(pathfinding_node)
    ld.add_action(aquabot_launch_file)

    return ld
