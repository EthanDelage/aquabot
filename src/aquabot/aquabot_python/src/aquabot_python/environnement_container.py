# This class contains all informations about environnement

def generate_environnement(obstacles=True):
    # Generate environnement container
    no_go_zones = []
    if obstacles:
        no_go_zones.append(EnvironnementNoGoZone((120, -50), 25)) # aquabot_lighthouse_island
        no_go_zones.append(EnvironnementNoGoZone((-152, -6), 50)) # aquabot_island_1
        no_go_zones.append(EnvironnementNoGoZone((110, 130), 50)) # aquabot_island_2
        no_go_zones.append(EnvironnementNoGoZone((12, -102), 25)) # aquabot_rock_island_0
        no_go_zones.append(EnvironnementNoGoZone((92, 170), 25)) # aquabot_rock_island_1
        no_go_zones.append(EnvironnementNoGoZone((-92, 176), 30)) # aquabot_rock_0
        no_go_zones.append(EnvironnementNoGoZone((-40, 220), 30)) # aquabot_rock_1
        no_go_zones.append(EnvironnementNoGoZone((-44, -95), 30)) # aquabot_rock_2
        no_go_zones.append(EnvironnementNoGoZone((-30, -150), 30)) # aquabot_rock_3
    return EnvironnementContainer((0, 0), 300, no_go_zones, obstacles)

class EnvironnementNoGoZone:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

class EnvironnementContainer:
    def __init__(self, starting_position, max_range, no_go_zones, obstacles=True):
        # Starting point and map borders
        self.starting_position = starting_position
        self.map_borders = (self.starting_position[0]-max_range, self.starting_position[0]+max_range, 
            self.starting_position[1]-max_range, self.starting_position[1]+max_range)
        # No go zones
        self.no_go_zones = no_go_zones
        self.obstacles = obstacles

