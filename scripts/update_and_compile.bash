#!/bin/bash


export LANG=en_US.UTF-8
export LANG=en

wstool merge /tmp/irp6.rosinstall
wstool update
cd underlay_isolated
catkin config --cmake-args -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_CORE_ONLY=ON -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON
catkin build

if [ $? -eq 0 ]; 
then 
	echo "underlay_isolated build OK" 
else 
	echo "underlay_isolated build FAILED" 
	exit 1 
fi
cd ../underlay
catkin config --extend ../underlay_isolated/devel/ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF
catkin build

if [ $? -eq 0 ]; 
then 
	echo "underlay build OK" 
else 
	echo "underlay build FAILED" 
	exit 1 
fi

cd ../robot
catkin config --extend ../underlay/devel/ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF
catkin build

if [ $? -eq 0 ]; 
then 
	echo "robot build OK" 
else 
	echo "robot build FAILED" 
	exit 1 
fi
