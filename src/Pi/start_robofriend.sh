# cd mjpg
mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so -x 320 -y 240 -quality 30 --fps 15" &
source ~/.bashrc
python3 start_robofriend.py

