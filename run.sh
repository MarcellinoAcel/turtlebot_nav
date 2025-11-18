#!/bin/bash

gnome-terminal --tab --title="Robot" -- bash -c "source install/setup.bash;
                                                 ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py localization:=true nav2:=true rviz:=true;
                                                 echo Press any key to close;
                                                 read -n 1"






