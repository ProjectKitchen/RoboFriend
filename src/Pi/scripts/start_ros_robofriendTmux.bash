#!/bin/bash

sudo /home/pi/robofriend_SW/RoboFriend/src/Pi/iowarrior/iow 7 0 0 100
source ~/ros_catkin_ws/devel/setup.bash
cd /home/pi/robofriend_SW/RoboFriend/src/Pi/catkin_ws/
source devel/setup.bash
cd src/robofriend_wrapper/src
ROS_ROOT='pwd'

tmux new-session -d -s robofriend

### Video Stream
tmux new-window -n 'mjpeg Streamer' -t robofriend /home/pi/Desktop/start_mjpegStream.sh

### roscore + lidar
tmux new-window -n 'lidar' -t robofriend roslaunch robofriend_wrapper robofriend.launch

### tf stuff
tmux new-window -n 'tf' -t robofriend rosrun tf static_transform_publisher -0.07 0.0 0.65 0 0 0 base_link laser 100

### robofriend base-controller
tmux new-window -n 'base controller' -t robofriend rosrun robofriend_basecontroller robofriend_basecontroller.py

### robofriend odom
tmux new-window -n 'odom' -t robofriend rosrun robofriend_odom robofriend_odom_pub.py

### robofriend main
tmux new-window -n 'robofriend wrapper' -t robofriend rosrun robofriend_wrapper robofriend.py 





