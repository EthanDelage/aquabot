from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    navigation_node = Node(
        package='navigation',
        executable='navigation',
    )
    pathfinding_node = Node(
        package='pathfinding',
        executable='pathfinding',
    )
    perception_node = Node(
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

    ld.add_action(navigation_node)
    ld.add_action(pathfinding_node)
    ld.add_action(perception_node)
    ld.add_action(aquabot_launch_file)

    return ld
