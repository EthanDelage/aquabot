# Aquabot competition

The Aquabot competition is a configuration of the VRX competition intended for an audience less knowledgeable about programming and for a shorter duration.


# System requierments for running aquabot simulation

Aquabot environnement is dependent on the VRX environment, the system requirements are the same.

### Hardware

In order to run VRX we recommend the following minimum hardware:
- Modern multi-core CPU, e.g. Intel Core i5
- 8 Gb of RAM
- Nvidia Graphics Card, e.g. Nvidia GTX 650

The system can be run without a dedicated GPU, but the Gazebo simulation will run much faster (should run in real-time) with access to a GPU.  Without a GPU the simulation is likely to run significantly slower than real-time.

### Software

RobotX teams provide good documentation for installing the environnement. [system setup tutorials](https://github.com/osrf/vrx/wiki/tutorials) 

The supported software environnement :
- Ubuntu Desktop 22.04 Jammy (64-bit)
- Gazebo Sim 7.0.0+
- ROS 2 Humble


# Installation

To build the VRX software, you need a development environment with the necessary dependencies installed (these include ROS, Gazebo and some utilities).

## Step 1 : Operating system (Ubuntu 22.04)

For this step you have 2 choices :
- Install Ubuntu 22.04 on Host machine [Ubuntu on host](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- Install Ubuntu 22.04 on Virtual machine [Ubuntu on vitualbox](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)

[Ubuntu 22.04.2 LTS Image](https://releases.ubuntu.com/jammy/)

A thrird choice would be to use [Docker](https://github.com/osrf/vrx/wiki/docker_install_tutorial) but i've not tested this one.

## Step 2 : ROS 2 Humble and Gazebo Garden

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Install [Gazebo Garden](https://gazebosim.org/docs/garden/install_ubuntu)

## Step 3 : Additional dependencies

```
sudo apt install python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro
```

## Step 4 : Installing VRX [Link](https://github.com/osrf/vrx/wiki/installation_tutorial)

Once you have set up your development environment, the following steps will download and build VRX:

Create a colcon workspace and clone the vrx repository
```
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx.git
```

Build the workspace
```
source /opt/ros/humble/setup.bash
cd ~/vrx_ws
colcon build --merge-install
```

Then you can now test the VRX simulation, follow [VRX tutorials](https://github.com/osrf/vrx/wiki/running_vrx_tutorial).

## Step 5 : Installing Aquabot 

Clone this respository inside "~/vrx_ws/src" folder.

That for, you need to setup your ssh connection on github. (follow [generate a new ssh-key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key) and [add a new ssh-key to you github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account#adding-a-new-ssh-key-to-your-account) ).
```
cd ~/vrx_ws/src
git clone git@github.com:sirehna/Aquabot.git
```
* Important : File path of this readme should be ~/vrx_ws/src/aquabot/README.md


Build :
```
cd ~/vrx_ws
colcon build --merge-install
. install/setup.bash
```

You can test aquabot simulation :
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```

## Common problems

### Errors building colcon workspace

Forgetting to source the environment is one of the most common mistakes among new users.

```
source /opt/ros/humble/setup.bash
cd ~/vrx_ws
colcon build --merge-install
```

### Warning depreciated "setup.py"

To avoid this warning you need to install python setuptools version 58.2.0 :

```
sudo apt install pip3
pip install setuptools==58.2.0
```

### Reduce simulation load

To reduce the simulation load gazebo can be limited by modifying 
the following parameters in the sdf file task (aquabot_gz/worlds/)

```sdf
<real_time_factor>0.0</real_time_factor>
<real_time_update_rate>0.0</real_time_update_rate>
```

[more details](https://craftsmumship.com/unlocking-the-real-time-factor-of-ros-gazebo-for-enhanced-robot-performance/)

For the evaluation we will use these default values

