from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='aquabot_task_hard'
    )
    node_arg = DeclareLaunchArgument(
        'node',
        default_value='pathfinding'
    )

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
            'world': world,
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        node_arg,
        navigation_node,
        pathfinding_node,
        perception_node,
        aquabot_launch_file,
    ])