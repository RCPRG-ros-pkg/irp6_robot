#!/bin/bash


export LANG=en_US.UTF-8
export LANG=en

wstool merge /tmp/irp6.rosinstall
wstool update
cd underlay_isolated
catkin_make_isolated --install -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB -DCMAKE_BUILD_TYPE=RelWithDebInfo
if [ $? -eq 0 ]; 
then 
	echo "underlay_isolated build OK" 
else 
	echo "underlay_isolated build FAILED" 
	exit 1 
fi
source install_isolated/setup.bash
cd ../underlay
catkin_make_isolated -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF --install
if [ $? -eq 0 ]; 
then 
	echo "underlay build OK" 
else 
	echo "underlay build FAILED" 
	exit 1 
fi

source install_isolated/setup.bash
cd ../robot
catkin_make_isolated -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF --install
if [ $? -eq 0 ]; 
then 
	echo "robot build OK" 
else 
	echo "robot build FAILED" 
	exit 1 
fi
source install_isolated/setup.bash
