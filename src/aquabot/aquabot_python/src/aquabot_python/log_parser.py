from aquabot_python.replay_container import ReplayContainer, ReplayColumns, ReplayPhase
from aquabot_python.scenario_container import ScenarioContainer, TrajectoryType, TrajectoryPoint, TrajectoryPath
from aquabot_python.environnement_container import generate_environnement
from aquabot_python.scenario_pyplot import plot_scenario, PlotPoint

import matplotlib.pyplot as plt

def plot_column(replay, column):
    plt.plot(replay.get_column_time(), 
             replay.get_column_from_enum(column), 
             label=replay.get_column_name(column))
    
def plot_graphs(replay, save_path='replay_graphs.png', show=False):
    plt.figure(figsize=(10, 6))

    plt.subplot(2, 2, 1)
    plot_column(replay, ReplayColumns.Buoy_Distance)
    plt.axhline(y=20, color='yellow', linestyle='--', label='Buoy tolerance distance') # Plot yellow line at 20 m (tolerance distance)
    plt.xlabel('Time (s)')
    plt.title('Distance à la bouée de recherche en fonction du temps')
    try:
        rally_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.RALLY.value)
        time_start = replay.get_column_from_enum(ReplayColumns.Time)[rally_phase_index]
        plt.axvline(x=time_start, color='green', linestyle='--', label='Start rally phase')
    except ValueError:
        print("Phase RALLY not in replay")
    try:
        alert_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.ALERT.value)
        time_end = replay.get_column_from_enum(ReplayColumns.Time)[alert_phase_index]
        plt.axvline(x=time_end, color='red', linestyle='--', label='End rally phase')
    except ValueError:
        print("Phase ALERT not in replay")
    plt.legend()

    plt.subplot(2, 2, 2)
    plot_column(replay, ReplayColumns.Last_Alert_Error)
    plot_column(replay, ReplayColumns.Alert_RMS_Error)
    plt.title('Evolution de l\'erreur de l\'alerte en fonction du temps')
    try:
        alert_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.ALERT.value)
        time_start = replay.get_column_from_enum(ReplayColumns.Time)[alert_phase_index]
        time_end = time_start+180
        plt.axvline(x=time_start, color='green', linestyle='--', label='Start alert phase')
        plt.axvline(x=time_end, color='red', linestyle='--', label='End alert phase')
    except ValueError:
        print("Phase ALERT not in replay")
    plt.legend()

    plt.subplot(2, 2, 4)
    plot_column(replay, ReplayColumns.Last_Follow_Error)
    plot_column(replay, ReplayColumns.Follow_RMS_Error)
    plt.title('Evolution de l\'erreur de suivi en fonction du temps')
    try:
        alert_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.ALERT_FOLLOW.value)
        time_start = replay.get_column_from_enum(ReplayColumns.Time)[alert_phase_index]
        time_end = replay.get_column_from_enum(ReplayColumns.Time)[-1]
        plt.axvline(x=time_start, color='green', linestyle='--', label='Start follow phase')
        plt.axvline(x=time_end, color='red', linestyle='--', label='End follow phase')
    except ValueError:
        print("Phase ALERT_FOLLOW not in replay")
    plt.legend()

    plt.subplot(2, 2, 3)
    plot_column(replay, ReplayColumns.Ennemy_Distance)
    plot_column(replay, ReplayColumns.Ally0_Distance)
    plot_column(replay, ReplayColumns.Ally1_Distance)
    plt.axhline(y=10, color='red', linestyle='--', label='Collision min distance') # Plot red line at 10 m (min distance)
    plt.title('Distance aux obstacles en fonction du temps')
    plt.legend()

    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    plt.tight_layout()
    plt.savefig(save_path, dpi=400)
    if show:
        plt.show()

def get_trajectory_path_from_replay(replay, column_x, column_y, column_yaw, type):
    time = replay.get_column_time()
    x = replay.get_column_from_enum(column_x)
    y = replay.get_column_from_enum(column_y)
    if(column_yaw is None):
        yaw = [0] * len(time)
    else:
        yaw = replay.get_column_from_enum(column_yaw)
    path = []
    for i in range(len(time)):
        path.append(TrajectoryPoint([x[i], y[i]], yaw[i], time[i]))
    return TrajectoryPath(type, path)

def plot_map(replay, obstacles, save_path="map.png", show=True):
    trajectories = []
    trajectories.append(get_trajectory_path_from_replay(replay, 
        ReplayColumns.Player_X, ReplayColumns.Player_Y, ReplayColumns.Player_Yaw, TrajectoryType.PLAYER))
    trajectories.append(get_trajectory_path_from_replay(replay,
        ReplayColumns.Ennemy_X, ReplayColumns.Ennemy_Y, ReplayColumns.Ennemy_Yaw, TrajectoryType.ENEMY))
    trajectories.append(get_trajectory_path_from_replay(replay,
        ReplayColumns.Ally0_X, ReplayColumns.Ally0_Y, ReplayColumns.Ally0_Yaw, TrajectoryType.ALLY))
    trajectories.append(get_trajectory_path_from_replay(replay,
        ReplayColumns.Ally1_X, ReplayColumns.Ally1_Y, ReplayColumns.Ally1_Yaw, TrajectoryType.ALLY))
    trajectories.append(get_trajectory_path_from_replay(replay,
        ReplayColumns.Alert_Position_X, ReplayColumns.Alert_Position_Y, None, TrajectoryType.ALERT))
    scenario = ScenarioContainer([replay.get_column_from_enum(ReplayColumns.Buoy_X)[0],
                                  replay.get_column_from_enum(ReplayColumns.Buoy_Y)[0]],
                                  trajectories)
    point_list = [] #  triangle = ralliement bouée, étoile = première alerte, square: début de phase poursuite, cercle = début d’émission position menace
    # Try get rally phase position : Get line index of first phase ALERT and then rally position
    try:
        alert_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.ALERT.value)
        last_rally_player_position = [replay.get_column_from_enum(ReplayColumns.Player_X)[alert_phase_index],
                          replay.get_column_from_enum(ReplayColumns.Player_Y)[alert_phase_index]]
        last_rally_ennemy_position = [replay.get_column_from_enum(ReplayColumns.Ennemy_X)[alert_phase_index],
                            replay.get_column_from_enum(ReplayColumns.Ennemy_Y)[alert_phase_index]]
        point_list.append(PlotPoint(last_rally_ennemy_position[0], last_rally_ennemy_position[1], 'Ennemy position at rally', '^', 'red', 'darkred'))
        point_list.append(PlotPoint(last_rally_player_position[0], last_rally_player_position[1], 'Player position at rally', '^', 'blue', 'darkblue'))
    except ValueError:
        print("Phase ALERT not found in replay")
    # Try get ennemy and player position at ALERT_FOLLOW phase : Get line index of first phase ALERT_FOLLOW and then ennemy position
    try:
        alert_follow_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.ALERT_FOLLOW.value)
        alert_follow_ennemy_position = [replay.get_column_from_enum(ReplayColumns.Ennemy_X)[alert_follow_phase_index],
                          replay.get_column_from_enum(ReplayColumns.Ennemy_Y)[alert_follow_phase_index]]
        alert_follow_player_position = [replay.get_column_from_enum(ReplayColumns.Player_X)[alert_follow_phase_index],
                            replay.get_column_from_enum(ReplayColumns.Player_Y)[alert_follow_phase_index]]
        point_list.append(PlotPoint(alert_follow_ennemy_position[0], alert_follow_ennemy_position[1], 'Ennemy position at start follow phase', 's', 'red', 'darkred'))
        point_list.append(PlotPoint(alert_follow_player_position[0], alert_follow_player_position[1], 'Player position at start follow phase', 's', 'blue', 'darkblue'))
    except ValueError:
        print("Phase ALERT_FOLLOW not found in replay")
    # Try get ennemy and player position at FOLLOW phase : Get line index of first phase FOLLOW and then ennemy position
    try:
        follow_phase_index = replay.get_column_from_enum(ReplayColumns.Phase).tolist().index(ReplayPhase.FOLLOW.value)
        follow_ennemy_position = [replay.get_column_from_enum(ReplayColumns.Ennemy_X)[follow_phase_index],
                          replay.get_column_from_enum(ReplayColumns.Ennemy_Y)[follow_phase_index]]
        follow_player_position = [replay.get_column_from_enum(ReplayColumns.Player_X)[follow_phase_index],
                            replay.get_column_from_enum(ReplayColumns.Player_Y)[follow_phase_index]]
        point_list.append(PlotPoint(follow_ennemy_position[0], follow_ennemy_position[1], 'Ennemy position at last phase', 'o', 'red', 'darkred'))
        point_list.append(PlotPoint(follow_player_position[0], follow_player_position[1], 'Player position at last phase', 'o', 'blue', 'darkblue'))
    except ValueError:
        print("Phase FOLLOW not found in replay")
    # Try get first alert and ennemy position : Get line index of first alert and then alert position
    try:
        first_alert_index = replay.get_column_from_enum(ReplayColumns.Alert_RMS_Number).tolist().index(1)
        first_alert_position = [replay.get_column_from_enum(ReplayColumns.Alert_Position_X)[first_alert_index],
                            replay.get_column_from_enum(ReplayColumns.Alert_Position_Y)[first_alert_index]]
        first_ennemy_position = [replay.get_column_from_enum(ReplayColumns.Ennemy_X)[first_alert_index],
                            replay.get_column_from_enum(ReplayColumns.Ennemy_Y)[first_alert_index]]
        point_list.append(PlotPoint(first_alert_position[0], first_alert_position[1], 'First alert position', '*', 'orange', 'darkorange'))
        point_list.append(PlotPoint(first_ennemy_position[0], first_ennemy_position[1], 'Ennemy position at first alert', '*', 'red', 'darkblue'))
    except ValueError:
        print("First alert not found in replay")
    # Plot scenario
    plot_scenario(scenario, generate_environnement(obstacles), save_path=save_path, show=show, point_list=point_list)

def plot_log_csv(filepath, obstacles=True):
    replay = ReplayContainer(filepath)
    plot_graphs(replay, "replay_graphs.png", show=True)
    plot_map(replay, obstacles, "replay_map.png", show=True)
    