#!/bin/bash

run_option=$*

source /opt/tdt/setupvars.sh
source ./install/setup.bash
ros2 run $run_option
