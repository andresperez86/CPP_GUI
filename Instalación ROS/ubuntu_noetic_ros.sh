#!/bin/bash

## Bash script for setting up ROS Melodic (with Gazebo 9) development environment for PX4 on Ubuntu LTS (18.04).
## It installs the common dependencies for all targets (including Qt Creator)
##
## Installs:
## - Common dependencies libraries and tools as defined in `ubuntu_sim_common_deps.sh`
## - ROS Melodic (including Gazebo9)
## - MAVROS

if [[ $(lsb_release -sc) == *"bionic"* ]]; then
  echo "OS version detected as $(lsb_release -sc) (18.04)."
  echo "ROS Melodic requires at least Ubuntu 20.04."
  echo "Exiting ...."
  return 1;
fi

echo "Downloading dependent script 'ubuntu_sim_common_deps.sh'"
# Source the ubuntu_sim_common_deps.sh script directly from github
common_deps=$(wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'ubuntu_sim_common_deps.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
. <(echo "${common_deps}")

# ROS Melodic
## Gazebo simulator dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/melodic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt update
## Get ROS/Gazebo
sudo apt install ros-noetic-desktop-full
# Environment setup
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Dependencies for building packages
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# Initialize rosdep
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/noetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Install dependencies
# sudo apt-get install python-catkin-tools python-rosinstall-generator -y is for ubuntu 18.04
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y

# 1. Create the workspace: unneeded if you already has workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src

# 2. Install MAVLink
#    we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall

# 3. Install MAVROS: get source (upstream - released)
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

# 4. Create workspace & deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

# 5. Install GeographicLib datasets:
sudo apt install geographiclib-tools -y
echo "Downloading dependent script 'install_geographiclib_datasets.sh'"
# Source the install_geographiclib_datasets.sh script directly from github
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'install_geographiclib_datasets.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
sudo bash -c "$install_geo"

# 6. Build source
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source

