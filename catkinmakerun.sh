#!/bin/bash
cd ../../..
catkin_make
cd src/CSE_190_PA2/scripts
roslaunch cse_190_assi_2 solution_python.launch
