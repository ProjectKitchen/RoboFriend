source SET_ROS_MASTER.sh
source /opt/ros/melodic/setup.bash


tmux new-session -d -s robofriend

#MAP SERVER
xterm -e "rosrun map_server map_server /home/adrian/GIT/RoboFriend/src/RemotePC/maps/new_map_cor.yaml" &

source ../catkin_ws/devel/setup.bash
#move_base
xterm -e "roslaunch robofriend_navstack amcl.launch" &

#source ROBOFRIEND_SW1/RoboFriend/src/Pi/catkin_ws/devel/setup.bash

xterm -e "roslaunch robofriend_navstack move_base.launch" &


xterm -e "rosrun robofriend_charging_station calc_drive_commands.py" &

xterm -e "rosrun robofriend_charging_station calcCahrignStationPos_ownImp.py" &


xterm -e "rosrun robofriend_set_goal robofriend_set_goal.py" &

xterm -e "rosrun robofriend_charging_station cs_Commander.py"&
#rviz
xterm -e "rviz" &
#rqt
xterm -e "rqt" &
