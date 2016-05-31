#!/bin/bash

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/irp6_robot/master/scripts/init.bash -O /tmp/init.bash

bash /tmp/init.bash
if [ $? == 1 ]
then
	exit
fi


if [ ! -d $1 ]; then
  mkdir $1
fi

cp var.cfg /tmp

cd $1
wstool init

bash /tmp/update_and_compile.bash
cp var.cfg $1/robot/src/irp6_robot/scripts
