#!/bin/bash
cd ~/catkin_ws/src
git clone https://github.com/blakebiz/robotics_report3
git clone https://github.com/blakebiz/robotics_lab4
git clone https://github.com/blakebiz/robotics_lab6
cd ..
catkin_make
