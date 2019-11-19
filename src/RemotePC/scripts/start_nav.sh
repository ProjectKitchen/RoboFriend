source SET_ROS_MASTER.sh
source /opt/ros/melodic/setup.bash

#MAP SERVER
xterm -e "rosrun map_server map_server /home/adrian/Robofriend/new_map.yaml " &

source ../catkin_ws/devel/setup.bash
#move_base
xterm -e "roslaunch robofriend_navstack amcl.launch" &

#source ROBOFRIEND_SW1/RoboFriend/src/Pi/catkin_ws/devel/setup.bash

xterm -e "roslaunch robofriend_navstack move_base.launch" &


#rviz

