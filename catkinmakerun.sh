#!/bin/bash
cd ../../..
catkin_make
cd src/cse_190_pa2/scripts
roslaunch cse_190_assi_2 solution_python.launch
