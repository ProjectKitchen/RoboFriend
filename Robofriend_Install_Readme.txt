
Using Legacy Raspian Buster (because of MJpeg Streamer / Raspicam compatibility)!
User Image:

Prepare System:
setup Wlan
Preferences -> Screen Configuration -> 1920 x 1080 
sudo apt update
sudo apt upgrade
sudo raspi-config (-> Interface Options -> P1 Enable Camera)

Install Arduino:
(https://www.arduino.cc/en/software/)
wget https://downloads.arduino.cc/arduino-1.8.19-linuxarm.tar.xz

Install SerialCommand library:
https://github.com/scogswell/ArduinoSerialCommand

Install Teensyduino:
(https://www.pjrc.com/teensy/td_download.html)
wget https://www.pjrc.com/teensy/td_159/TeensyduinoInstall.linuxarm
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
chmod 755 TeensyduinoInstall.linuxarm
./TeensyduinoInstall.linuxarm (Install Teensyduino, select TimerThree library)

Build / Install MJpeg Streamer
(https://github.com/jacksonliam/mjpg-streamer)
git clone https://github.com/jacksonliam/mjpg-streamer.git
sudo apt-get install cmake libjpeg8-dev
cd mjpg-streamer-experimental
make
sudo make install

Optional: test mjpeg streamer:
export LD_LIBRARY_PATH=.
./mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so"

Install Robofriend Source Repo & Dependencies
(https://github.com/ProjectKitchen/RoboFriend)
git clone https://github.com/ProjectKitchen/RoboFriend.git
sudo apt-get install espeak
python -m pip install pyttsx3




deal with xcb/threading issue:
https://discuss.wxpython.org/t/xcb-unknown-request-in-queue-while-dequeuing/26830/4