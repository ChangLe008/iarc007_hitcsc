#!/bin/bash
start_ros(){
	source /opt/ros/indigo/setup.bash
	source /home/hitcsc/.bashrc
	source /home/hitcsc/catkin_ws/devel/setup.sh
	#echo "hitcsc" | sudo -s roslaunch dji_sdk sdk_manifold.launch
	cd /home/hitcsc/catkin_ws/devel/lib/dji_sdk
	roslaunch dji_sdk sdk_manifold.launch
}

	start_ros
	#echo "hitcsc" | sudo -s roslaunch dji_sdk sdk_manifold.launch
exit 0
