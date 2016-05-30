#!/bin/bash
# Skrypt powinien być wołany z katalogu robot/src/irp6_robot/scripts

echo "Reading config...." >&2

if test -e ./var.cfg; then
	source ./var.cfg
else
	touch var.cfg
	echo "Please fill the var.cfg file in scripts directory with variables values"
	exit
fi

echo "Config for the ros_version: $ros_version" >&2
echo "Config for the hardware_mode: $hardware_mode" >&2

if [ "x$ROS_MASTER_URI" != "x" ]
then
	echo "Ta konsola jest skazona, otworz nowa bez source'owania zadnych plikow ROSa"
	exit
fi


if [ "$ros_version" == "jade" ]
then
	source /opt/ros/jade/setup.bash
elif [ "$ros_version" == "kinetic" ]
then
	source /opt/ros/kinetic/setup.bash
else 
	echo "w pliku var.cfg ustaw ros_version=\"jade\" albo ros_version=\"kinetic\""
	exit
fi

if [ "$hardware_mode" == "true" ]
then
	 wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/irp6.rosinstall -O /tmp/irp6.rosinstall

elif [ "$hardware_mode" == "false"  ]
then
	wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/irp6_sim.rosinstall -O /tmp/irp6.rosinstall
else 
	echo "w pliku var.cfg ustaw hardware_mode=\"true\" albo hardware_mode=\"false\""
	exit
fi


# wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash


cd ../../../../

bash /tmp/update_and_compile.bash