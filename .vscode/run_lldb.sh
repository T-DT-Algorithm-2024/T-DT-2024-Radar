#!/bin/bash

run_option=$*

source ./install/setup.bash

if [ -f /usr/bin/gnome-terminal ]
then
    gnome-terminal --command "ros2 run --prefix 'lldb-server platform --server --listen *:3039' $run_option"
else
    if [ -f /usr/bin/konsole ]
    then
        konsole -e "ros2 run --prefix 'lldb-server platform --server --listen *:3039' $run_option"
    fi
fi

