#!/bin/bash

if [ "x$ROS_MASTER_URI" == "x" ]
then
	echo "Please source ros"
	exit
fi

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/irp6_sim.rosinstall -O /tmp/irp6.rosinstall
wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash

if [ ! -d $1 ]; then
  mkdir $1
fi

cd $1
wstool init

bash /tmp/update_and_compile.bash

