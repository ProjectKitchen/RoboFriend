# 360 laser distance sensor lds-01
sudo apt-get install ros-kinetic-hls-lfcd-lds-driver
# dependent packages for TurtleBot3 
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
# get git submodules
cd ~/Git/RoboFriend/src/Pi/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
cd ~/Git/RoboFriend/src/Pi/catkin_ws/src/hls_lfcd_lds_driver
git checkout melodic-devel
# clean and build the catkin workspace
cd ~/Git/RoboFriend/src/Pi/catkin_ws/ && catkin_make clean && catkin_make && catkin_make install
