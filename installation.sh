#!/bin/bash
# Script to install Spiri Simulator

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-hydro-desktop-full

sudo apt-get install ros-hydro-hector-quadrotor

sudo apt-get install ros-hydro-ros-controllers

sudo apt-get install ros-hydro-joy


# Configuring ros workspace

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

catkin_init_workspace

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Download Spiri simulator

cd ~/catkin_ws/src

wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/installation_simulator/Simulator-1.0.tar.gz

tar -zxvf Simulator-1.0.tar.gz 

mv -r Simulator-1.0 ./Simulator

cd ..

catkin_make

# Set Gazebo Model Path

echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Simulator/spiri_description/models" >> ~/.bashrc




