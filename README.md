# 808X_FinalProject
Final project of software development course. Turtlebot can explore an unknown world autonomously.

[![Build Status](https://travis-ci.org/SonamYeshe/808X_FinalProject.svg?branch=master)](https://travis-ci.org/SonamYeshe/808X_FinalProject)
[![Coverage Status](https://coveralls.io/repos/github/SonamYeshe/808X_FinalProject/badge.svg?branch=master)](https://coveralls.io/github/SonamYeshe/808X_FinalProject?branch=master)
---

## License
GNU General Public License v3.0

Permissions of this strong copyleft license are conditioned on making available complete source code of licensed works and modifications, which include larger works using a licensed work, under the same license. Copyright and license notices must be preserved. Contributors provide an express grant of patent rights.

## Disclaimer 
Jiawei Ge(SonamYeshe), hereby disclaims all copyright interest in the program `walker' (which makes passes at compilers) written by Jiawei Ge(SonamYeshe).

 (signature of Jiawei Ge), 14 November 2017

 Jiawei Ge

## Dependencies
1. ROS kinetic
2. Gazebo 7.0.0
3. RViz version 1.12.13 (kinetic)
4. Travis CI
5. Coveralls.io
6. ROS packages: stage_ros, rviz, slam_gmapping, move_base, 
gazebo_ros, turtlebot_navigation, turtlebot_gazebo, rviz, nodelet.

## Backlog google spreadsheet
https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/1Yvd1-wER0aqfgY3TEJp908nOtQcfq_xC7EuNYKqGUYw/edit?usp=sharing

## run the world
```
$ roslaunch finalproject turtlebot_world.launch world_file:=`rospack find finalproject`/worlds/corridor.world
$ roslaunch finalproject turtlebot_world.launch world_file:=`rospack find finalproject`/worlds/willowgarage.world
```

roslaunch finalproject nav_stack.launch

## Doxygen
$ sudo apt install doxygen


