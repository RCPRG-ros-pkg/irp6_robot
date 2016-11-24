#!/usr/bin/env bash


if [[ $(rosnode list | grep diagnostic_aggregator) ]] ; then
	echo "Common alrady running"
else
	roslaunch irp6_bringup irp6-common.launch
fi

