# This class contains all informations about scenario for aquabot competition
from enum import Enum

class TrajectoryType(Enum):
    ENEMY = 1
    ALLY = 2
    PLAYER = 3
    ALERT = 4

class TrajectoryPoint:
    def __init__(self, point, yaw, time):
        self.point = point
        self.yaw = yaw
        self.time = time

class TrajectoryPath:
    def __init__(self, type, path):
        self.path = path
        self.type = type

class ScenarioContainer:
    def __init__(self, buoy_position, trajectory_paths):
        self.buoy_position = buoy_position
        self.trajectory_paths = trajectory_paths
