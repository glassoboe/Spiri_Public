#Simulator

## For the latest relase please go to the following link

https://github.com/Pleiades-Spiri/Spiri_Public/tree/Simulator-alpha-1.1

The latest instructions are in the Readme.md. 

## Introduction

This branch contains the alpha release of the simulator.

It contains the following functionalities-:

- Spawn Spiri in Gazebo with a choice of a world

- Publish sensor data on individual topics.

- Control Spiri in the simulator by a joystick.

The code has been tested on ROS Hydro and Ubuntu 12.04

## System Requirements

Currently the Simulator works only on Ubuntu. ROS Hydro is offically supported for Ubuntu 12.04, 12.10, 13.04. 

We are in the process of testing the Simulator on other variants of Linux distributions, Windows and Mac. 

## Installation instructions

You can install using scripts on Ubuntu 12.04 or install ROS, Gazebo independently. 

### Install using a script on Ubuntu 12.04

```bash
wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/installation_simulator/installation.sh
wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/installation_simulator/installation_simulator.sh
chmod +x installation.sh installation_simulator.sh
./installation.sh
```
Restart the terminal and run 
```bash
./installation_simulator.sh
```
This will install ROS, create a ROS workspace as well as download the Simulator code.

It will require your password to install packages. 

Jump to the command section.

### Install different components independently 

If you have installed using a script you dont need to do this. If you already have ROS and Gazebo installed jump to "Get the Simulator" section.

- Install ROS

http://wiki.ros.org/hydro/Installation/Ubuntu

Install the ros-desktop-full package as it contains Gazebo and other programs required for the simulator.

- Install Gazebo sensor plugins

sudo apt-get install ros-hydro-hector-quadrotor

- Install the Controllers

sudo apt-get install ros-hydro-ros-controllers

- Install joystick drivers

sudo apt-get install ros-hydro-joy

- Create a ros workspace

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

After this step there should be a folder called as catkin_ws

- Get the Simulator 

```bash
cd catkin_ws/src

wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/installation_simulator/Simulator-1.0.tar.gz

tar -zxvf Simulator-1.0.tar.gz 

mv Simulator-1.0 ./Simulator

cd ..

catkin_make

echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Simulator/spiri_description/models" >> ~/.bashrc

source ~/.bashrc
```

These commands will build the code required for the Simulator.

Make sure these commands are in your bashrc
```bash

nano ~/.bashrc

source ~/catkin_ws/devel/setup.bash

source /opt/ros/hydro/setup.bash

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Simulator/spiri_description/models
```

## Commands
Please restart the terminal and then you can launch all the commands.

To launch Spiri in an empty world

```bash
roslaunch spiri_description spiri_empty_world.launch
```

To launch Spiri in Citadel Hill

```bash
roslaunch spiri_description spiri_citadel.launch
```

Control Spiri with a joystick

Xbox controller 

```bash
roslaunch spiri_teleop xbox_controller.launch
```

Logitech Gamepad

```bash
roslaunch spiri_teleop logitech_gamepad.launch
```  


Script to process the Depth Map

```bash
ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True _queue_size:=10
```

View the Depth Map

```bash
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color _approximate_sync:=True _queue_size:=10
```

View the front left camera

```bash
rosrun image_view image_view image:=/stereo/left/image_raw
```

View the front right camera

```bash
rosrun image_view image_view image:=/stereo/right/image_raw
```

View the bottom facing camera

```bash
rosrun image_view image_view image:=/downward_cam/camera/image
```

## To visualize the robot and point cloud data in rviz

```bash
roscd spiri_description

rosrun rviz rviz -d spiri_pointcloud.rviz
```

## Nodes

### gazebo
Launches spiri in a world with all its sensors

#### Subscribed topics 

- /cmd_vel
- /wind
#### Published Topics

- /altimeter
- /fix
- /fix/pose
- /ground_truth/state
- /raw_imu
- /pressure_height
- /magnetic
- /stereo/left/image_raw
- /stereo/right/image_raw

### quadrotor_teleop
Control Spiri with joystick

#### Subscribed topics

- /joy

#### Published topics

- /cmd_vel


## Troubleshooting

### Error [ConnectionManager.cc:116] Failed to connect to master in 30 seconds

This is a known bug in Gazebo

https://bitbucket.org/osrf/gazebo/issue/732/gazebo-never-starts-when-loading-models

To fix this do the following steps-:

```bash
cd catkin_ws/src

hg clone https://bitbucket.org/osrf/gazebo_models

Add this command to your bashrc

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/gazebo_models
```

or you can run this script

```bash

wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/installation_simulator/gazebo_models.sh
chmod +x gazebo_models.sh
./gazebo_models.sh

```
Restart your terminal and it should fix the error

## General ROS commands

To list all ROS topics

```bash
rostopic list
```

To view any ROS topic 
```bash
rostopic echo topic_name
```
For example to view altimeter data

```bash
rostopic echo /altimeter
```
