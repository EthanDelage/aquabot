ROS_SOURCE = /opt/ros/humble/setup
VRX_SOURCE = install/setup

BASH_ROS_SOURCE = $(ROS_SOURCE).bash
BASH_VRX_SOURCE = $(VRX_SOURCE).bash
ZSH_VRX_SOURCE = $(VRX_SOURCE).zsh
ZSH_ROS_SOURCE = $(ROS_SOURCE).zsh

.PHONY: run
run:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_task_hard > /dev/null &

.PHONY:	build
build:
	colcon build --base-paths src/ --merge-install
	@echo "\e[31mPlease run \". $(ZSH_VRX_SOURCE)\" before running simulation"

.PHONY: fclean
fclean:
	$(RM) -r install/ log/ build/

.PHONY: re
re:	fclean
	$(MAKE) build
	$(MAKE) run

.PHONY: launch
launch:
	ros2 launch launcher launch.py

.PHONY: perception
perception:
	ros2 run perception perception

.PHONY: navigation
navigation:
	ros2 run navigation navigation

.PHONY: pathfinding
pathfinding:
	ros2 run pathfinding pathfinding

.PHONY: headless
headless:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_task_hard headless:=true > /dev/null &

.PHONY: teleop
teleop:
	ros2 run aquabot_python teleop_keyboard.py

.PHONY: rqt
rqt:
	rqt &

.PHONY: rviz
rviz:
	ros2 launch aquabot_gz rviz.launch.py > /dev/null &

.PHONY: debug
debug:
	$(MAKE) run
	$(MAKE) rviz
	$(MAKE) rqt

.PHONY: add_shell_source
add_shell_source:
	echo "# Source ROS Humble" >> ~/.bashrc
	echo "source $(BASH_ROS_SOURCE)" >> ~/.bashrc
	echo "# Source ROS Humble" >> ~/.zshrc
	echo "source $(ZSH_ROS_SOURCE)" >> ~/.zshrc
	
