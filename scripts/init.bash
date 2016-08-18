#!/bin/bash


echo "Reading config...." >&2

if test -e ./var.cfg; then
	source ./var.cfg
else
	touch var.cfg
	echo "Please fill the var.cfg file in scripts directory with variables values"
	exit 1
fi

echo "Config for the ros_version: $ros_version" >&2
echo "Config for the hardware_mode: $hardware_mode" >&2

if [ "x$ROS_MASTER_URI" != "x" ]
then
	echo "Ta konsola jest skazona, otworz nowa bez source'owania zadnych plikow ROSa"
	exit 1
fi


if [ "$hardware_mode" == "true" ]
then
	wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/stable/scripts/irp6_hardware_kinetic.rosinstall -O /tmp/irp6.rosinstall

elif [ "$hardware_mode" == "false"  ]
then
	wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/stable/scripts/irp6_sim_kinetic.rosinstall -O /tmp/irp6.rosinstall
else 
	echo "w pliku var.cfg ustaw hardware_mode=\"true\" albo hardware_mode=\"false\""
	exit 1
fi


wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/stable/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash

exit 0


