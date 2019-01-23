# cd mjpg
mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so -x 320 -y 240 -quality 30 --fps 15" &
# build catkin workspace
# cd catkin_ws/ && catkin_make clean && catkin_make && catkin_make install && cd ..
source ~/.bashrc
python3 start_robofriend.py

