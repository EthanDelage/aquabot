# Aquabot_competitor

# Installation

Before preoceed to this installation you first needs to install vrx and aquabot simulation.

## Step 1 : Aquabot repository

Follow the aquabot installation steps first :
[Aquabot repository](https://github.com/sirehna/Aquabot)

## Step 2 : Additional dependencies

Install OpenCV dependency :
```
sudo apt install libopencv-dev python3-opencv
```

## Step 3 : Aquabot_competitor

Clone this respository inside "~/vrx_ws/src" folder
* Important : File path of this readme should be ~/vrx_ws/src/aquabot_competitor/README.md

Build :
```
cd ~/vrx_ws
colcon build --merge-install
. install/setup.bash
```

# Commands

List of helpful commands and tips.

**Creating your participation package**

To create a package you can create a sub folder and implement the 2 nessessary files (CMakeLists.txt and package.xml) or simply use the next command.

```
cd ~/vrx_ws/src/aquabot_competitor
ros2 pkg create --build-type ament_cmake --node-name my_team_node my_team_package
```

**Compile our package :**
```
cd ~/vrx_ws
colcon build --merge-install
. install/setup.bash
```

**Run our node (C++ or Python):**

To run a ros node you can use 'ros2 run' command :
```
ros2 run package_example example_node_cpp
ros2 run package_example example_node.py
```

**Monitor our topics using commands**
In an other terminal you can read our "status_string" topic :
```
source /opt/ros/humble/setup.bash
ros2 topic echo status_string
```

**Monitor our topics using RQT**
RQT is a powerfull toolbox that contains many ros tools :
```
source /opt/ros/humble/setup.bash
rqt
```

**Monitor our topics using rviz**
RVIZ2 is a powerfull visualization tool that provides a 3D graphical interface to visualize and 
interact with ros environnement, such as sensor data, robot models, and trajectories. 
It allows users to monitor and debug their robot systems in a user-friendly and intuitive manner.

After start a simulation you can start rviz2 :
```
rviz2
```
Then you can press Add -> By topic and select what you want to see.

**Start the competition simulation**

Without the gazebo graphical environnement. (lightweight) I recommend it for non GPU configurations.


## Permanent sourcing and aliases

I share my additions to the "bashrc" file :

```
gedit ~/.bashrc
```

Add these lines have everythink sourced when starting new terminal :
```
# Source ROS Humble
source /opt/ros/humble/setup.bash
# Source VRX Workspace
source ~/vrx_ws/install/setup.bash
```

Add these lines to add 3 alias to build, source and code for the project :
```
# Add Alias
alias aquabot_build='cd ~/vrx_ws && colcon build --merge-install'
alias aquabot_source='source ~/vrx_ws/install/setup.bash'
alias aquabot_code='code ~/vrx_ws/src/aquabot_competitor'
```


## Example content (aquabot_examples):

This project already contains example content inside aquabot_example folder
- package_example
- opencv_example
- aquabot_example

These are 3 package that contains cpp and python nodes that can serve as examples packages.

### package_example

This is a simple package example, contain cpp and python node examples. 

You can refer to this ROS tutorial :
Link : https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

### opencv_example

This is a opencv example package, you can refer to this example for opencv implementation.

Contains 2 nodes :
CPP Node : minimal_opencv_ros2_node
Creates and publish random_image

Python Node : minimal_opencv_subscriber
Subscribe to the random_image topic and show the image

### aquabot_example

This is an example applied to the aquabot competition

Contains 1 node :
CPP Node : That moves the motor and retrieve that position
