from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, EmitEvent
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import aquabot_gz.bridges
import vrx_gz.bridges

import os

PATROLANDFOLLOW_WORLDS = [
  'aquabot_task_easy',
  'aquabot_task_medium',
  'aquabot_task_hard'
]

def competition_bridges(world_name, competition_mode=False):
    bridges = [
        vrx_gz.bridges.clock(),
        vrx_gz.bridges.task_info()
    ]

    if not competition_mode:
        bridges.extend([
            vrx_gz.bridges.usv_wind_speed(),
            vrx_gz.bridges.usv_wind_direction()
        ])

    task_bridges = []
    if world_name in PATROLANDFOLLOW_WORLDS:
        task_bridges = [
            aquabot_gz.bridges.patrolandfollow_current_phase(),
            aquabot_gz.bridges.patrolandfollow_ais_ennemy_position(),
            aquabot_gz.bridges.patrolandfollow_ais_ennemy_speed(),
            aquabot_gz.bridges.patrolandfollow_ais_allies_positions(),
            aquabot_gz.bridges.patrolandfollow_ais_allies_speeds(),
            aquabot_gz.bridges.patrolandfollow_alert_position(),
        ]
        if not competition_mode:
            task_bridges.extend([
                aquabot_gz.bridges.patrolandfollow_debug_buoy_pose_error(),
                aquabot_gz.bridges.patrolandfollow_debug_alert_pose_error(),
                aquabot_gz.bridges.patrolandfollow_debug_follow_pose_error(),
                aquabot_gz.bridges.patrolandfollow_debug_alert_mean_error(),
                aquabot_gz.bridges.patrolandfollow_debug_follow_mean_error(),
                aquabot_gz.bridges.patrolandfollow_debug_ennemy_distance(),
                aquabot_gz.bridges.patrolandfollow_debug_allies_distance(),
            ])
            
    bridges.extend(task_bridges)

    nodes = []
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    ))
    return nodes
