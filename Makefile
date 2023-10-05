ROS_SOURCE = /opt/ros/humble/setup
VRX_SOURCE = install/setup

BASH_ROS_SOURCE = $(ROS_SOURCE).bash
BASH_VRX_SOURCE = $(VRX_SOURCE).bash
ZSH_ROS_SOURCE = $(ROS_SOURCE).zsh

.PHONY: run
run:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta > /dev/null &

.PHONY:	build
build:
	colcon build --merge-install
	bash -c ". $(BASH_VRX_SOURCE)"

.PHONY: fclean
fclean:
	$(RM) -r install/ log/ build/

.PHONY: re
re:	fclean
	$(MAKE) build
	$(MAKE) run

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
