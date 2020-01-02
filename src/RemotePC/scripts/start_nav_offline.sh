#source SET_ROS_MASTER.sh
source /opt/ros/melodic/setup.bash

xterm -e "roscore" &

sleep 2

rosparam set use_sim_time true

#MAP SERVER
xterm -e "rosrun map_server map_server /home/adrian/GIT/RoboFriend/src/RemotePC/maps/new_map_cor.yaml" &

source ../catkin_ws/devel/setup.bash
#move_base
xterm -e "roslaunch robofriend_navstack amcl.launch" &

#source ROBOFRIEND_SW1/RoboFriend/src/Pi/catkin_ws/devel/setup.bash

xterm -e "roslaunch robofriend_navstack move_base.launch" &

#rviz
xterm -e "rviz" &

#rosbag
xterm -e "rosbag play --clock ../OdomScan.bag" &

source ../../Pi/catkin_ws/devel/setup.bash
#robofriend tf
xterm -e "rosrun tf static_transform_publisher -0.07 0.0 0.65 0 0 0 base_link laser 100" &

#robofriend odom
xterm -e "rosrun robofriend_odom robofriend_odom_pub.py" &
