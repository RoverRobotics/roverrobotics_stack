# About this stack

A ROS1 stack that handles communication between Rover Robotics rover platforms and computer payloads.
The rover platform accept velocity commands and publishes wheel encoder data and power information.

Documentation for this package can be found at http://wiki.ros.org/rr_openrover_stack

Master|Develop|Kinetic-Devel
---|---|---
[![CircleCI](https://circleci.com/gh/RoverRobotics/rr_openrover_stack/tree/master.svg?style=svg)](https://circleci.com/gh/RoverRobotics/rr_openrover_stack/tree/master)|[![CircleCI](https://circleci.com/gh/RoverRobotics/rr_openrover_stack/tree/develop.svg?style=svg)](https://circleci.com/gh/RoverRobotics/rr_openrover_stack/tree/develop)|[![CircleCI](https://circleci.com/gh/RoverRobotics/rr_openrover_stack/tree/kinetic-devel.svg?style=svg)](https://circleci.com/gh/RoverRobotics/rr_openrover_stack/tree/kinetic-devel)

# Robot Computer Installation Guide

This guide is to help you setup your Computer with Rover Packages to able to run Rover Robotics Robots

## Prerequisites / Requirements
1. Ubuntu 18.0.4
2. Git
3. Nano
4. ROS Melodic
5. Rosdep
## Installation
Open a Terminal and Navigate to your ROS Workspace

Install missing ros packages with
```bash
sudo apt install ros-melodic-joy ros-melodic-serial ros-melodic-twist-mux ros-melodic-tf2-geometry-msgs ros-melodic-robot-localization ros-melodic-gmapping ros-melodic-move-base -y
```
Go to src folder
```bash
cd src/
```
Use the git to clone our repository

```bash
git clone https://github.com/roverrobotics/rr_openrover_stack rr_openrover_stack
```
Clone ps4 ROS driver(optional). Need Driver from Below if install.
```bash
git clone https://github.com/naoki-mizuno/ds4_driver
```

In order to consistently detect the robots every reboot, we need to setup the Udev rules for it. Default UDEV rules are in our our stack. You can copy it via
```bash
sudo cp src/rr_openrover_stack/udev/roverrobotics.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
## Installing Dual Shock 4 (Ps4 Controller Driver)
The following steps are to help you 
install DualShock 4 (Ps4 Controller Driver) that being use as a default method for our robots. You may skip this step if you are using another controller.

Make a new directory to store the driver.
```bash
mkdir ~/drivers
cd ~/drivers
```
Clone and Install Ds4driver
```bash
git clone https://github.com/naoki-mizuno/ds4drv --branch devel
mkdir -p ~/.local/lib/python2.7/site-packages
python2 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
```
Now you should be able to build your ROS Workspace

Navigate to ROS workspace. 

Run this to update your dependencies.
```bash
rosdep install --from-paths src --ignore-src -r -y
```
Build and source your workspace
```bash
catkin_make
source /devel/setup.bash
```

## Usage

Your Robot can now be control via our Launch Files. *** These are template launch files. You may need to add or remove nodes if your robot are configured with a different hardware***

Rover Pro:
```bash
roslaunch rr_openrover_driver teleop.launch
```
Rover Pro with slam pack:
```bash
roslaunch rr_openrover_driver slampack.launch
```

Rover Zero:
```bash
roslaunch rr_roverzero_driver teleop.launch
```
Rover Zero V2:
```bash
roslaunch rr_rover_zero_v2_driver teleop.launch
```
# Other Installation

## Systemd startup scripts
We have prepared some system startup scripts that would automaticly launch the robot to be able to control with the ps4 controller on power up. 

This is useful for those who just want to drive the robot around.

Run this command to inside your ROS Workspace install the startup script
```bash
sudo src/rr_openrover_stack/opscripts/install.sh
