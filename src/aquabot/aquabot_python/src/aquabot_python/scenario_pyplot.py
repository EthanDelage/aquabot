import matplotlib.pyplot as plt

from aquabot_python.scenario_container import ScenarioContainer, TrajectoryType, TrajectoryPoint, TrajectoryPath
from aquabot_python.environnement_container import EnvironnementContainer

import math
import os

ARROW_SIZE = 1

class PlotPoint:
    def __init__(self, x, y, label="", marker='o', color='black', markerfacecolor='black', fillstyle="full", markersize=8):
        self.x = x
        self.y = y
        self.label = label
        self.marker = marker
        self.color = color
        self.markerfacecolor = markerfacecolor
        self.fillstyle = fillstyle
        self.markersize = markersize
    
    def plot(self):
        plt.plot(self.x, self.y, label=self.label, marker=self.marker, color=self.color, markerfacecolor=self.markerfacecolor, fillstyle=self.fillstyle, markersize=self.markersize)

def plot_background(obstacles, map_borders):
    # Get env variable COLCON_PREFIX_PATH
    install_path = os.environ['COLCON_PREFIX_PATH']
    # Get image : From install_path/share/aquabot_python/images
    if obstacles:
        im = plt.imread(os.path.join(install_path, 'share', 'aquabot_python', 'images', 'map_300.png'))
    else:
        im = plt.imread(os.path.join(install_path, 'share', 'aquabot_python', 'images', 'map_no_obstacles_300.png'))
    # Plot image
    plt.imshow(im, extent = map_borders)

def plot_trajectory(trajectory_path):
    # Choose color from type
    color = 'blue'
    if trajectory_path.type == TrajectoryType.ENEMY:
        color = 'red'
    elif trajectory_path.type == TrajectoryType.ALLY:
        color = 'green'
    elif trajectory_path.type == TrajectoryType.PLAYER:
        color = 'blue'
    elif trajectory_path.type == TrajectoryType.ALERT:
        color = 'orange'
    # Plot first point
    plt.plot(trajectory_path.path[0].point[0], trajectory_path.path[0].point[1], 'x', color=color)
    # Plot trajectory
    x, y = [], []
    for point in trajectory_path.path:
        if trajectory_path.type == TrajectoryType.ALERT and point.point[0] == 0 and point.point[1] == 0:
            continue
        x.append(point.point[0])
        y.append(point.point[1])
        # Plot arrow yaw
        if point.yaw != 0:
            plt.arrow(point.point[0], point.point[1], 
                  ARROW_SIZE*math.cos(point.yaw),
                  ARROW_SIZE*math.sin(point.yaw),
                  color='gray')
    plt.plot(x, y, color=color)

def plot_scenario(scenario_container, environnement_container, save_path='scenario.png', show=False, point_list=[]):
    # Clean plot 
    plt.clf()
    # Plot background
    plot_background(environnement_container.obstacles, environnement_container.map_borders)
    # Plot buoy
    plt.plot(scenario_container.buoy_position[0], scenario_container.buoy_position[1], 'o', color='purple', label='Buoy', markersize=10)
    # Plot trajectories
    for trajectory_path in scenario_container.trajectory_paths:
        plot_trajectory(trajectory_path)
    # Plot map borders
    plt.xlim(environnement_container.map_borders[0], 
             environnement_container.map_borders[1])
    plt.ylim(environnement_container.map_borders[2], 
             environnement_container.map_borders[3])
    # Plot no go zones
    for no_go_zone in environnement_container.no_go_zones:
        circle = plt.Circle(no_go_zone.center, no_go_zone.radius, color='grey', fill=False, alpha=0.5)
        plt.gcf().gca().add_artist(circle)
    # Plot starting point
    plt.plot(environnement_container.starting_position[0],
                environnement_container.starting_position[1], 'x', color='blue')
    # Plot point list
    for point in point_list:
        point.plot()
    # Show legend on top right outside plot
    plt.legend(bbox_to_anchor=(1, 1), loc='upper left')
    #Configure plot
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    # Save plot
    plt.savefig(save_path, dpi=400)
    # Show plot
    if show:
        plt.show()
    
