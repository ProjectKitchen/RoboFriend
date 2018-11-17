#cd mjpg
mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so -x 320 -y 280 -quality 30 --fps 15 -vf -rot 180" &
#..cd 
#python3 robofriend.py


