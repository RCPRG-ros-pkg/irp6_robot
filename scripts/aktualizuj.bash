#!/bin/bash
# Skrypt powinien być wołany z katalogu robot/src/irp6_robot/scripts

cp ../../../../.rosinstall ../../../../.rosinstall.bk2
cp ../../../../.rosinstall.bak ../../../../.rosinstall


wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/stable/scripts/init.bash -O /tmp/init.bash

bash /tmp/init.bash

if [ $? -eq 1 ]
then
	exit
fi

source ./var.cfg
source /opt/ros/kinetic/setup.bash

cd ../../../../

bash /tmp/update_and_compile.bash
if [ $? -eq 1 ]
then
	exit
fi
