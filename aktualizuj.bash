#!/bin/bash

if [ "x$ROS_MASTER_URI" != "x" ]
then
	echo "Ta konsola jest skazona, otworz nowa bez source'owania zadnych plikow ROSa"
	exit
fi

source /opt/ros/indigo/setup.bash
export LANG=en
if [ -d ~/ws_irp6 ]
then
	cd ~/ws_irp6
	wstool up
	cd underlay_isolated
	catkin_make_isolated --install -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB -DCMAKE_BUILD_TYPE=RelWithDebInfo
	source install_isolated/setup.bash
	cd ../underlay
	catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF
	source devel/setup.bash
fi

if [ -d ~/catkin_ws ]
then
	cd ~/catkin_ws
	rm -rf build/ devel/
	catkin_make
fi



