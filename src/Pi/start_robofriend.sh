# cd mjpg
mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so -x 320 -y 240 -quality 30 --fps 15" &
# clean and build the catkin workspace
cd ~/Git/RoboFriend/src/Pi/catkin_ws/ && catkin_make clean && catkin_make && catkin_make install
source ~/.bashrc
python3 start_robofriend.py

