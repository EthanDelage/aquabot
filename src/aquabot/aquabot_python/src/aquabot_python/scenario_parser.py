# This is xml parser for scenario file
import xml.etree.ElementTree as ET
from aquabot_python.scenario_container import ScenarioContainer, TrajectoryPath, TrajectoryType, TrajectoryPoint

BUOY_NAME = 'aquabot_sonar_buoy'
ACTOR_ENEMY = 'ennemy'
ACTOR_ALLY = 'ally_'

def deserialize(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    world = root.find('world')
    buoy_position = parse_buoy_position(world)
    trajectory_paths = parse_trajectory_paths(world)
    return ScenarioContainer(buoy_position, trajectory_paths)

def parse_buoy_position(root):
    # Buoy position is stored in include/pose tag
    for include in root.findall('include'):
        name = include.find('name').text
        if name == BUOY_NAME:
            pose = include.find('pose').text
            pose = pose.split()
            return [float(pose[0]), float(pose[1])]
    print ("ERROR: Buoy position not found in scenario file")
    exit(1)

def parse_trajectory_paths(root):
    # Actor tag contains trajectory path
    trajectory_paths = []
    for actor in root.findall('actor'):
        name = actor.get('name')
        if name == ACTOR_ENEMY:
            trajectory_paths.append(parse_trajectory_path(actor, TrajectoryType.ENEMY))
        elif name.startswith(ACTOR_ALLY):
            trajectory_paths.append(parse_trajectory_path(actor, TrajectoryType.ALLY))
    return trajectory_paths

def parse_trajectory_path(actor, type):
    # Trajectory path is stored in script/trajectory tag
    trajectory_path = TrajectoryPath(type, [])
    trajectory = actor.find('script').find('trajectory')
    for waypoint in trajectory.findall('waypoint'):
        time = float(waypoint.find('time').text)
        pose = waypoint.find('pose').text
        pose = pose.split()
        trajectory_path.path.append(TrajectoryPoint([float(pose[0]), float(pose[1])], float(pose[5]), time))
    return trajectory_path

def serialize(scenario_container, file_path, seed=None):
    # NCH test working with boxes
    # NCH add real path to dae
    # NCH check form adding collisions and add it to the actors
    # Then check to fix the orientation of the actors
    sdf = ET.Element('sdf')
    root = ET.SubElement(sdf, 'world')
    # Seed
    if seed is not None:
        ET.SubElement(root, 'generation_seed').text = str(seed)
    # Platform
    platform = ET.SubElement(root, 'include')
    ET.SubElement(platform, 'name').text = 'platform'
    ET.SubElement(platform, 'uri').text = 'platform'
    # Yellow buoy
    include = ET.SubElement(root, 'include')
    ET.SubElement(include, 'name').text = BUOY_NAME
    ET.SubElement(include, 'pose').text = '%f %f 0 0 1.57 0' % (scenario_container.buoy_position[0], scenario_container.buoy_position[1])
    ET.SubElement(include, 'uri').text = BUOY_NAME
    plugin = ET.SubElement(include, 'plugin')
    plugin.set('name', 'vrx::PolyhedraBuoyancyDrag')
    plugin.set('filename', 'libPolyhedraBuoyancyDrag.so')
    ET.SubElement(plugin, 'fluid_density').text = '1000'
    ET.SubElement(plugin, 'fluid_level').text = '0.0'
    ET.SubElement(plugin, 'linear_drag').text = '25.0'
    ET.SubElement(plugin, 'angular_drag').text = '2.0'
    buoyancy = ET.SubElement(plugin, 'buoyancy')
    buoyancy.set('name', 'collision_outer')
    ET.SubElement(buoyancy, 'link_name').text = 'link'
    ET.SubElement(buoyancy, 'pose').text = '0 0 0 0 -1.57 0'
    geometry = ET.SubElement(buoyancy, 'geometry')
    cylinder = ET.SubElement(geometry, 'cylinder')
    ET.SubElement(cylinder, 'radius').text = '1.65'
    ET.SubElement(cylinder, 'length').text = '0.05'
    wavefield = ET.SubElement(plugin, 'wavefield')
    ET.SubElement(wavefield, 'topic').text = '/vrx/wavefield/parameters'
    # Pinger
    plugin = ET.SubElement(root, 'plugin')
    plugin.set('filename', 'libPublisherPlugin.so')
    plugin.set('name', 'vrx::PublisherPlugin')
    message = ET.SubElement(plugin, 'message')
    message.set('type', 'gz.msgs.Vector3d')
    message.set('topic', '/pinger/set_pinger_position')
    message.set('at', '1.0')
    message.text = 'x: %f, y: %f, z: 0' % (scenario_container.buoy_position[0], scenario_container.buoy_position[1])
    # Actors
    id_ally = 0
    for trajectory_path in scenario_container.trajectory_paths:
        actor = ET.SubElement(root, 'actor')
        if trajectory_path.type == TrajectoryType.ENEMY:
            actor.set('name', ACTOR_ENEMY)
        elif trajectory_path.type == TrajectoryType.ALLY:
            actor.set('name', ACTOR_ALLY + str(id_ally))
            id_ally += 1
        skin = ET.SubElement(actor, 'skin')
        if trajectory_path.type == TrajectoryType.ENEMY:
            ET.SubElement(skin, 'filename').text = '../models/aquabot_redboat/mesh/redboat.dae'
        elif trajectory_path.type == TrajectoryType.ALLY:
            ET.SubElement(skin, 'filename').text = '../models/aquabot_greenboat/mesh/greenboat.dae'
        script = ET.SubElement(actor, 'script')
        ET.SubElement(script, 'loop').text = 'true'
        if trajectory_path.type == TrajectoryType.ENEMY:
            ET.SubElement(script, 'delay_start').text = '60'
        elif trajectory_path.type == TrajectoryType.ALLY:
            ET.SubElement(script, 'delay_start').text = '20'
        ET.SubElement(script, 'auto_start').text = 'true'
        trajectory = ET.SubElement(script, 'trajectory')
        trajectory.set('id', '0')
        trajectory.set('type', 'square')
        for trajectory_point in trajectory_path.path:
            waypoint = ET.SubElement(trajectory, 'waypoint')
            ET.SubElement(waypoint, 'time').text = '%f' % trajectory_point.time
            ET.SubElement(waypoint, 'pose').text = '%f %f -0.1 0 0 %f' % (trajectory_point.point[0], trajectory_point.point[1], trajectory_point.yaw)
    # Scoring
    scoring = ET.SubElement(root, 'plugin')
    scoring.set('filename', 'libPatrolAndFollowScoringPlugin.so')
    scoring.set('name', 'vrx::PatrolAndFollowScoringPlugin')
    ET.SubElement(scoring, 'vehicle').text = 'wamv'
    ET.SubElement(scoring, 'task_name').text = 'patrolandfollow'
    ET.SubElement(scoring, 'task_info_topic').text = '/vrx/task/info'
    ET.SubElement(scoring, 'initial_state_duration').text = '10'
    ET.SubElement(scoring, 'ready_state_duration').text = '10'
    ET.SubElement(scoring, 'running_state_duration').text = '900'
    ET.SubElement(scoring, 'release_topic').text = '/vrx/release'
    ET.SubElement(scoring, 'pinger_position').text = '%f %f 0' % (scenario_container.buoy_position[0], scenario_container.buoy_position[1])
    ET.SubElement(scoring, 'target_name').text = ACTOR_ENEMY
    markers = ET.SubElement(scoring, 'markers')
    ET.SubElement(markers, 'scaling').text = '0.2 0.2 2.0'
    ET.SubElement(markers, 'height').text = '0.5'
    ET.SubElement(scoring, 'phase_rally_tolerance').text = '20'
    ET.SubElement(scoring, 'phase_alert_process_period').text = '1.0'
    ET.SubElement(scoring, 'phase_alert_tolerance').text = '50.0'
    ET.SubElement(scoring, 'phase_follow_process_period').text = '1.0'
    ET.SubElement(scoring, 'phase_follow_tolerance').text = '50'
    ET.SubElement(scoring, 'phase_follow_target_distance').text = '30'
    ET.SubElement(scoring, 'phase_follow_ennemy_time').text = '180.0'
    ET.SubElement(scoring, 'collision_min_distance').text = '10.0'
    ET.SubElement(scoring, 'collision_finish').text = 'true'
    ET.SubElement(scoring, 'collision_penality').text = '10'
    no_collisions_objects = ET.SubElement(scoring, 'collision_objects')
    id_ally = 0
    for trajectory_path in scenario_container.trajectory_paths:
        if trajectory_path.type == TrajectoryType.ALLY:
            object = ET.SubElement(no_collisions_objects, 'object')
            ET.SubElement(object, 'name').text = ACTOR_ALLY + str(id_ally)
            id_ally += 1
    ET.SubElement(scoring, 'ais_allies_period').text = '1.0'
    ET.SubElement(scoring, 'ais_ennemy_period').text = '10.0'
    ET.SubElement(scoring, 'ais_ennemy_wait_time').text = '180.0'
    # Write to file
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space="  ")
    tree.write(file_path, encoding='utf-8', xml_declaration=False)
