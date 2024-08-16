#!/bin/bash

run_option=$*

source ./install/setup.bash
gnome-terminal -- "ros2 launch $run_option"
