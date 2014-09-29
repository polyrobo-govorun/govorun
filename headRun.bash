#!/bin/bash
roscore &
#export ROS_PACKAGE_PATH="~/Videos/faceCommandSystem/faceCommandSystem/:$ROS_PACKAGE_PATH"
#export ROS_PACKAGE_PATH="/home/polyrobo/faceCommandSystem:$ROS_PACKAGE_PATH"
export ROS_PACKAGE_PATH="/home/polyrobo/dev/faceCommandSystem:$ROS_PACKAGE_PATH"

roslaunch robohead robohead.launch &
roslaunch robohead kinect.launch &
rosrun robohead headController.py &
rosrun faceCommandSystem neckControlCamera &

#rosservice call /bottom_tilt_controller/set_speed -- 0.5
#rosservice call /upper_tilt_controller/set_speed -- 0.5
