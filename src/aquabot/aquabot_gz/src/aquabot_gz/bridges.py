
from vrx_gz.bridge import Bridge, BridgeDirection

## Competition topics
# GZ to ROS
def patrolandfollow_current_phase():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/current_phase',
        ros_topic=f'/vrx/patrolandfollow/current_phase',
        gz_type='ignition.msgs.UInt32',
        ros_type='std_msgs/msg/UInt32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_ais_ennemy_position():
    return Bridge(
        gz_topic=f'/wamv/ais_sensor/ennemy_position',
        ros_topic=f'/wamv/ais_sensor/ennemy_position',
        gz_type='ignition.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_ais_ennemy_speed():
    return Bridge(
        gz_topic=f'/wamv/ais_sensor/ennemy_speed',
        ros_topic=f'/wamv/ais_sensor/ennemy_speed',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_ais_allies_positions():
    return Bridge(
        gz_topic=f'/wamv/ais_sensor/allies_positions',
        ros_topic=f'/wamv/ais_sensor/allies_positions',
        gz_type='ignition.msgs.Pose_V',
        ros_type='geometry_msgs/msg/PoseArray',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_ais_allies_speeds():
    return Bridge(
        gz_topic=f'/wamv/ais_sensor/allies_speeds',
        ros_topic=f'/wamv/ais_sensor/allies_speeds',
        gz_type='ignition.msgs.Float_V',
        ros_type='ros_gz_interfaces/msg/Float32Array',
        direction=BridgeDirection.GZ_TO_ROS)

# ROS to GZ
def patrolandfollow_alert_position():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/alert_position',
        ros_topic=f'/vrx/patrolandfollow/alert_position',
        gz_type='ignition.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.ROS_TO_GZ)

## Debug topics
def patrolandfollow_debug_buoy_pose_error():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/debug/buoy_pose_error',
        ros_topic=f'/vrx/patrolandfollow/debug/buoy_pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_debug_alert_pose_error():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/debug/alert_pose_error',
        ros_topic=f'/vrx/patrolandfollow/debug/alert_pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_debug_follow_pose_error():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/debug/follow_pose_error',
        ros_topic=f'/vrx/patrolandfollow/debug/follow_pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_debug_alert_mean_error():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/debug/alert_mean_error',
        ros_topic=f'/vrx/patrolandfollow/debug/alert_mean_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_debug_follow_mean_error():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/debug/follow_mean_error',
        ros_topic=f'/vrx/patrolandfollow/debug/follow_mean_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_debug_ennemy_distance():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/debug/ennemy_distance',
        ros_topic=f'/vrx/patrolandfollow/debug/ennemy_distance',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def patrolandfollow_debug_allies_distance():
    return Bridge(
        gz_topic=f'/vrx/patrolandfollow/debug/allies_distance',
        ros_topic=f'/vrx/patrolandfollow/debug/allies_distance',
        gz_type='ignition.msgs.Float_V',
        ros_type='ros_gz_interfaces/msg/Float32Array',
        direction=BridgeDirection.GZ_TO_ROS)