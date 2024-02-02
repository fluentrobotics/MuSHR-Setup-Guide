---
title: MuSHR Desktop Setup Guide - Fluent Robotics Lab
author: Jeeho Ahn
date: Jan. 2024
---

# PuSHR Setup Guide - Fluent Robotics Lab

The original PuSHR repository is developed for Ubuntu 18.04 with Python 2, and it will not work out of the box. This guide provides modification to upgrade the original PuSHR system to work on Ubuntu 20.04 with Python 3.

[Original PuSHR Git Repo](https://github.com/prl-mushr/pushr)

## Prerequisite

This guide assumes a MuSHR system installed as described in [Fluent Robotics Lab MuSHR Desktop Setup Guide](mushr_desktop_setup.md).

## The modified PuSHR repository

For simplicity, we will use a modified repository fitted for our use instead of the original PuSHR repository. Make sure you have an access to this repo.

[Modified PuSHR Repository](https://github.com/fluentrobotics/PuSHR-Noetic)

```
cd ~/catkin_ws/src
git clone https://github.com/fluentrobotics/PuSHR-Noetic
cd ~/catkin_ws && catkin_make
```

## Modify robot kinematic constraint
```
sed -i '/servo_min:/c\  servo_min: 0.0' $(rospack find mushr_sim)/config/vesc.yaml
sed -i '/servo_max:/c\  servo_max: 1.0' $(rospack find mushr_sim)/config/vesc.yaml
```

## Running PuSHR Simulation

Run the following commands in order.

```
roscore
roslaunch mushr_coordination mushr_coordination.launch cars_file:=4cars.yaml
roslaunch clcbs_ros clcbs_ros.launch
roslaunch mushr_pixelart_mpc multi_sim.launch
rviz -d $(rospack find clcbs_ros)/rviz/clcbs.rviz
roslaunch clcbs_ros init_clcbs.launch benchmark_file:=ex3.yaml map_server:=0
```

You should now see the robots moving on RViz.