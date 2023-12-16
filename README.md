# Aqua.Bot 2023

![](assets/aquabot_cover.png)

## About

Aqua.Bot is a challenge organized by Sirehna (Naval Group). The goal is to develop a program able to control an USV (Unmanned Surface Vehicule) that will monitor a maritime zone and communicate the position of intruding vessels.

The competition lasted 2 months, during which we developed two modules:

### Navigation
The navigation module is able to control the drone autonomously in the environment. The first step is to reach the buoy that defines the search area in which the threat is located.
Secondly, it is capable to maneuvere the surface drone while tracking the threat.

https://github.com/EthanDelage/aquabot/assets/50496792/67a0170c-d944-461a-99a4-69b9b991643f



### Perception
The perception module is able to detect the threat using the camera and estimate the distance using the lidar. The threat is recognized using color detection, and the distance is calculated by projecting the 3D points of the lidar onto the camera rendering. Using the distance and angle of the threat, the module is then able to calculate its GPS position.

https://github.com/EthanDelage/aquabot/assets/50496792/6377734d-59ac-47a8-b206-32733cc76101


Please refer to the report for further information :

- [PDF report](assets/report.pdf)
- [PowerPoint presentation](assets/presentation)

## Usage

### Dependencies

- OpenCV 4.8.1
- Eigen3
- C++ 17
- Ros Humble
- Gazebo Garden

### Installation

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Install [Gazebo Garden](https://gazebosim.org/docs/garden/install_ubuntu)

### Source Ros

```shell
source /opt/ros/humble/setup.sh
```

### Build

```shell
make build
. install/setup.sh
```

### Run

```shell
make launch
```

## Acknowledgments

After 2 months of research and hard work, we've been awarded **second place** in the competition!
We would like to thank Sirehna and Naval Group for organizing the competition and supporting us. 
