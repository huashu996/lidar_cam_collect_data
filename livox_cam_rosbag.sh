#!/bin/bash
gnome-terminal -- bash -c "cd /home/b/cxl/hikrobot-mvs-camera-ros && source devel/setup.bash && roslaunch hikrobot_camera hikrobot_camera_outdoor.launch"

sleep 2

gnome-terminal -- bash -c "cd /home/b/cxl/livox_ros && source devel/setup.bash && roslaunch livox_ros_driver livox_lidar_msg.launch"




