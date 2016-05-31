#!/bin/bash
# Skrypt powinien być wołany z katalogu robot/src/irp6_robot/scripts

bash init.bash
if [ $? == 1 ]
then
	exit
fi

cd ../../../../

bash /tmp/update_and_compile.bash
