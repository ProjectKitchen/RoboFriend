source SET_ROS_MASTER.sh
source /opt/ros/melodic/setup.bash

source ROBOFRIEND_SW1/RoboFriend/src/Pi/catkin_ws/devel/setup.bash

xterm -e "rviz" &
xterm -e "rqt" &

xterm -e "rosrun gmapping slam_gmapping" &
