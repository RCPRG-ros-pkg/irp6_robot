#!/bin/bash
# Skrypt powinien być wołany z katalogu robot/src/irp6_robot/scripts

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/stable/scripts/init.bash -O /tmp/init.bash

bash /tmp/init.bash

if [ $? -eq 1 ]
then
	exit
fi

source ./var.cfg
if [ "$ros_version" == "jade" ]
then
	source /opt/ros/jade/setup.bash
elif [ "$ros_version" == "kinetic" ]
then
	source /opt/ros/kinetic/setup.bash
fi

cd ../../../../

bash /tmp/update_and_compile.bash
if [ $? -eq 1 ]
then
	exit
fi
