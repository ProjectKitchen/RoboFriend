#cd mjpg
mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so -x 320 -y 240 -quality 30 --fps 15" &
#cd .. 
source /home/pi/.bashrc
python3 robofriend1.py

