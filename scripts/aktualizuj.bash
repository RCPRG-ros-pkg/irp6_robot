#!/bin/bash
# Skrypt powinien być wołany z katalogu robot/src/irp6_robot/scripts

if [ "x$ROS_MASTER_URI" != "x" ]
then
	echo "Ta konsola jest skazona, otworz nowa bez source'owania zadnych plikow ROSa"
	exit
fi

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/irp6.rosinstall -O /tmp/irp6.rosinstall
wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash


cd ../../../../

bash /tmp/update_and_compile.bash