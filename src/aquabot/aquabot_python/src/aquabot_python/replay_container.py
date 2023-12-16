# This class contains all informations about replay log file for aquabot competition
from enum import Enum, auto
import matplotlib.pyplot as plt
import numpy as np

# Enum pour définir les colonnes
class ReplayColumns(Enum):
    Time = 0
    Phase = auto()
    Player_X = auto()
    Player_Y = auto()
    Player_Yaw = auto()
    Player_Speed = auto()
    Player_Distance = auto()
    Ennemy_X = auto()
    Ennemy_Y = auto()
    Ennemy_Yaw = auto()
    Ennemy_Speed = auto()
    Ennemy_Distance = auto()
    Ally0_X = auto()
    Ally0_Y = auto()
    Ally0_Yaw = auto()
    Ally0_Speed = auto()
    Ally0_Distance = auto()
    Ally1_X = auto()
    Ally1_Y = auto()
    Ally1_Yaw = auto()
    Ally1_Speed = auto()
    Ally1_Distance = auto()
    Buoy_X = auto()
    Buoy_Y = auto()
    Buoy_Distance = auto()
    Last_Alert_Error = auto()
    Alert_RMS_Error = auto()
    Alert_RMS_Number = auto()
    Alert_Position_X = auto()
    Alert_Position_Y = auto()
    Alert_Latitude = auto()
    Alert_Longitude = auto()
    Last_Follow_Error = auto()
    Follow_RMS_Error = auto()
    Follow_RMS_Number = auto()

class ReplayPhase(Enum):
    INITIAL = 0
    RALLY = 1
    ALERT = 2
    ALERT_FOLLOW = 3
    FOLLOW = 4
    FINISHED = 5

class ReplayContainer:
    def __init__(self, csv_file, txt_file=None):
        # Load csv file to numpy array
        self.data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)

    def get_column_from_enum(self, column):
        return self.data[:, column.value]
    
    def get_column_time(self):
        return self.get_column_from_enum(ReplayColumns.Time)
    
    def get_column_name(self, column):
        return column.name.replace('_', ' ')


