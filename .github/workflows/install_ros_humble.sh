#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

sudo apt update && sudo apt install locales
 locale-gen en_US en_US.UTF-8
 update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Installer les packages
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key with apt.
sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
