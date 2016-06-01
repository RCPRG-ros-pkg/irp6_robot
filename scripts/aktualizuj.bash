#!/bin/bash
# Skrypt powinien być wołany z katalogu robot/src/irp6_robot/scripts

bash init.bash
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
