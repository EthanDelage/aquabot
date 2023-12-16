from aquabot_python.scenario_container import ScenarioContainer, TrajectoryType, TrajectoryPoint, TrajectoryPath
from aquabot_python.environnement_container import EnvironnementContainer, EnvironnementNoGoZone, generate_environnement
from aquabot_python.scenario_parser import serialize, deserialize
from aquabot_python.scenario_pyplot import plot_scenario
from aquabot_python.generator_methods import get_random_trajectory_path_secure, get_random_point_in_map, get_distance_between_points
import random
import string

def generate_scenario(prefix_name="scenario", seed=None, obstacles=True):
    # Generate a random seed if not sepcified
    if (seed is None):
        letters = string.ascii_lowercase + string.ascii_uppercase + string.digits
        seed = ''.join(random.choice(letters) for i in range(16))
    print("Generation seed : " + str(seed))
    random.seed(seed)
    # Create environnement container
    environnement_container = generate_environnement(obstacles)
    # Create scenario container and generate random point
    buoy_position, angle = get_random_point_in_map(environnement_container)
    #buoy_position = (-100.0, -200.0) # Use specific buoy position
    scenario_container = ScenarioContainer(buoy_position, [])
    if scenario_container.buoy_position is None:
        print("Error : generate_scenario() : scenario_container.buoy_position is None")
        exit()
    # Generate ennemy trajectory
    scenario_container.trajectory_paths.append(get_random_trajectory_path_secure(TrajectoryType.ENEMY, environnement_container, scenario_container))
    scenario_container.trajectory_paths.append(get_random_trajectory_path_secure(TrajectoryType.ALLY, environnement_container, scenario_container))
    scenario_container.trajectory_paths.append(get_random_trajectory_path_secure(TrajectoryType.ALLY, environnement_container, scenario_container))
    # Parse scenario to xml
    serialize(scenario_container, prefix_name + ".sdf", seed=seed)
    # Plot scenario
    plot_scenario(scenario_container, environnement_container, prefix_name + ".png")

def reopen_scenario(prefix_name="scenario", obstacles=True):
    # Parse scenario from xml
    scenario_container = deserialize(prefix_name + ".sdf")
    # Create environnement container
    environnement_container = generate_environnement(obstacles)
    # Plot scenario
    plot_scenario(scenario_container, environnement_container, prefix_name + "_reopened.png")

def regenerate_scenario_file(prefix_name="scenario", obstacles=True):
    # Parse scenario from xml
    scenario_container = deserialize(prefix_name + ".sdf")
    # Regenerate path time
    speed = 3.0 # m/s
    for trajectory_path in scenario_container.trajectory_paths:
        time = 0
        last_point = trajectory_path.path[0].point
        for i, trajectory_point in enumerate(trajectory_path.path):
            time_to_add = get_distance_between_points(last_point, trajectory_point.point) / speed
            if trajectory_path.type == TrajectoryType.ENEMY and i < 5: # Reduce ennemy speed for the first 2 lines
                time_to_add = time_to_add * 2.0
            time += time_to_add
            trajectory_point.time = time
            last_point = trajectory_point.point

    # Create environnement container
    environnement_container = generate_environnement(obstacles)

    # Parse scenario to xml
    serialize(scenario_container, prefix_name + "_regenerated.sdf")

    # Plot scenario
    plot_scenario(scenario_container, environnement_container, prefix_name + "_regenerated.png")

if __name__ == "__main__":
    generate_scenario()