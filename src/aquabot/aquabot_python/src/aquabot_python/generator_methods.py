import random
import numpy

from aquabot_python.scenario_container import TrajectoryType, TrajectoryPoint, TrajectoryPath
from aquabot_python.environnement_container import EnvironnementContainer
import math

# Constants
RETRY_MAX = 100

# Enum : PointFindType
from enum import Enum

class PointFindType(Enum):
    INSIDE_MAP_NGZ = 0
    DISTANCE_MIN_MAX = 1
    DISTANCE_ANGLE_MIN_MAX = 2

# Simple methods
def get_angle_between_points(point1, point2):
    return math.atan2(point2[1]-point1[1], point2[0]-point1[0])

def get_distance_between_points(point1, point2):
    return math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)

def is_point_in_map(map_cords, point):
    return point[0] >= map_cords[0] and point[0] <= map_cords[1] and point[1] >= map_cords[2] and point[1] <= map_cords[3]

def is_segement_intersecting_circle(circle_position, circle_radius, segment_start, segment_end):
    # Segment AB and circle center C
    AB = numpy.subtract(segment_end, segment_start)
    AC = numpy.subtract(circle_position, segment_start)
    # Projection of AC on AB
    j = numpy.dot(AC, AB) / numpy.dot(AB, AB)
    D = (segment_start[0]+j*AB[0], segment_start[1]+j*AB[1])
    AD = numpy.subtract(D, segment_start)
    # Solve k : AD = k*AB
    k = numpy.dot(AD, AB) / numpy.dot(AB, AB)
    # If k is between 0 and 1, the projection is on the segment
    if k >= 0 and k <= 1:
        # Distance between the projection and the circle center
        distance = numpy.linalg.norm(numpy.subtract(D, circle_position))
        return distance < circle_radius
    else:
        if k < 0:
            distance = numpy.linalg.norm(numpy.subtract(segment_start, circle_position))
        else:
            distance = numpy.linalg.norm(numpy.subtract(segment_end, circle_position))
        return distance < circle_radius

def is_inside_circle(circle_position, circle_radius, position):
    return (position[0]-circle_position[0])**2 + (position[1]-circle_position[1])**2 < circle_radius**2

def is_point_in_no_go_zones(no_go_zones, point):
    for no_go_zone in no_go_zones:
        if is_inside_circle(no_go_zone.center, no_go_zone.radius, point):
            return True
    return False


# Random point in map
# Conditions : point must not be in a no go zone
#              point must be in map
# Conditions not always necessary : point must be at a minimum/maximum distance from another point
#                                   point must be in a random angle from a previous point
def get_random_point_in_map(environnement_container, point_find_type=PointFindType.INSIDE_MAP_NGZ, origin_point=(0,0), yaw_point=0, distance_min_max=(0,0), angle_min_max=(0,2*math.pi)):
    count = 0
    angle = 0
    while True:
        count += 1
        if point_find_type == PointFindType.INSIDE_MAP_NGZ:
            point = (random.uniform(environnement_container.map_borders[0], environnement_container.map_borders[1]),
                        random.uniform(environnement_container.map_borders[2], environnement_container.map_borders[3]))
        if point_find_type == PointFindType.DISTANCE_MIN_MAX or point_find_type == PointFindType.DISTANCE_ANGLE_MIN_MAX:
            angle = yaw_point + (random.uniform(angle_min_max[0], angle_min_max[1]) * ((random.randint(0,1)*2)-1))
            distance = random.uniform(distance_min_max[0], distance_min_max[1])
            point = (origin_point[0] + distance*math.cos(angle),
                        origin_point[1] + distance*math.sin(angle))
        check = True
        if not is_point_in_map(environnement_container.map_borders, point):
            check = False
        if is_point_in_no_go_zones(environnement_container.no_go_zones, point):
            check = False
        if point_find_type == PointFindType.DISTANCE_MIN_MAX or point_find_type == PointFindType.DISTANCE_ANGLE_MIN_MAX:
            for i in range(len(environnement_container.no_go_zones)):
                if is_segement_intersecting_circle(environnement_container.no_go_zones[i].center, environnement_container.no_go_zones[i].radius, origin_point, point):
                    check = False
        if check:
            return point, angle
        if count > RETRY_MAX:
            print("Error : get_random_point_in_map() : count > %d" % RETRY_MAX)
            return None, None

# Random trajectory path
# Trajectory generation differ for enemy and ally
def get_random_trajectory_path(type, environnement_container, scenario_container):
    if type == TrajectoryType.ENEMY:
        distance_min_max_from_buoy = (50, 60)
        distance_min_max = (30, 60)
        yaw_min_max = (math.radians(10), math.radians(50))
        distance_total_min = 3 * 900 # 3 m/s * 15 min
        # Hard parmeters
        #distance_min_max = (20, 40)
        #yaw_min_max = (math.radians(20), math.radians(70))
    if type == TrajectoryType.ALLY:
        distance_min_max = (30, 60)
        yaw_min_max = (math.radians(10), math.radians(70))
        distance_total_min = 3 * 900 # 3 m/s * 15 min

    # Generate first point for each path
    if type == TrajectoryType.ALLY:
        point, angle = get_random_point_in_map(environnement_container)
        yaw = random.uniform(0, 2*math.pi)
    if type == TrajectoryType.ENEMY:
        point, angle = get_random_point_in_map(environnement_container, PointFindType.DISTANCE_MIN_MAX, 
                                        scenario_container.buoy_position, 0, distance_min_max_from_buoy)
        if point is None:
            print("Error : get_random_trajectory_path() : first point is None")
            return None
        yaw_buoy = yaw_min_max[1]  * ((random.randint(0,1)*2)-1) # + or - yaw max
        yaw = get_angle_between_points(point, scenario_container.buoy_position) + yaw_buoy

    # Generate path (list of points)
    path = []
    angles = []
    path.append(point)
    distance_total = 0
    while distance_total < distance_total_min:
        point, angle = get_random_point_in_map(environnement_container, PointFindType.DISTANCE_ANGLE_MIN_MAX, 
                                            path[-1], yaw, distance_min_max, yaw_min_max)
        if point is None:
            print("Error : get_random_trajectory_path() : new point is None")
            return None
        path.append(point)
        angles.append(angle)
        yaw = angle
        distance_total += get_distance_between_points(path[-2], path[-1])

    trajectory_path = TrajectoryPath(type, [])
    
    # Generate trajectory (with intermediate points, yaw and time)
    distance_before_rotate = 10 # m
    speed = 3 # m/s
    time = 0 # s
    for i in range(len(path)-1):
        yaw = angles[i]
        intermediate_point = (path[i+1][0]-(distance_before_rotate*math.cos(yaw)), 
                              path[i+1][1]-(distance_before_rotate*math.sin(yaw)))
        distance_to_intermediate_point = get_distance_between_points(path[i], intermediate_point)
        trajectory_path.path.append(TrajectoryPoint(path[i], yaw, time))
        if type == TrajectoryType.ENEMY and i < 2: # Reduce ennemy speed for the first 2 lines
            time += distance_to_intermediate_point / speed * 2.0
        else:
            time += distance_to_intermediate_point / speed
        trajectory_path.path.append(TrajectoryPoint(intermediate_point, yaw, time))
        if type == TrajectoryType.ENEMY and i < 2: # Reduce ennemy speed for the first 2 lines
            time += distance_before_rotate / speed * 2.0
        else:
            time += distance_before_rotate / speed
    trajectory_path.path.append(TrajectoryPoint(path[-1], yaw, time))
    return trajectory_path

def get_random_trajectory_path_secure(type, environnement_container, scenario_container):
    count = 0
    while True:
        count += 1
        trajectory_path = get_random_trajectory_path(type, environnement_container, scenario_container)
        if trajectory_path is not None:
            return trajectory_path
        if count > RETRY_MAX:
            print("Error : get_random_trajectory_path_secure() : count > %d" % RETRY_MAX)
            exit()
