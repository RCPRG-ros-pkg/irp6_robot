#!/bin/bash

bash init.bash

if [ ! -d $1 ]; then
  mkdir $1
fi

cp var.cfg /tmp

cd $1
wstool init

bash /tmp/update_and_compile.bash
cp var.cfg $1/robot/src/irp6_robot/scripts
