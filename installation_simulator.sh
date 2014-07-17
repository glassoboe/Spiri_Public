#!/bin/bash
source ~/.bashrc

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

catkin_init_workspace

cd ..

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init

rosdep update
# Download Spiri simulator

cd ~/catkin_ws/src

wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/Simulator-alpha-1.1/Simulator-1.1.tar.gz

tar -zxvf Simulator-1.1.tar.gz 

mv Simulator-1.1 ./Simulator

cd ..

#Build the system

catkin_make

# Set Gazebo Model Path

echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Simulator/spiri_description/models" >> ~/.bashrc


