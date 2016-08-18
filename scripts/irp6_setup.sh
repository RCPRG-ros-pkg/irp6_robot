#!/bin/bash

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/stable/scripts/init.bash -O /tmp/init.bash

bash /tmp/init.bash
if [ $? == 1 ]
then
	exit
fi

source ./var.cfg
source /opt/ros/kinetic/setup.bash

if [ ! -d $1 ]; then
  mkdir $1
fi

cp var.cfg /tmp

cd $1
wstool init

bash /tmp/update_and_compile.bash
if [ $? -eq 1 ]
then
	exit
fi

cp /tmp/var.cfg $1/robot/src/irp6_robot/scripts
