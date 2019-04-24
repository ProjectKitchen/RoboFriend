# Robofriend

Robofriend - a semi-autonomous, remotely controllable robot companion.

Robofriend - initially created by Tom Heike 2002 - was exhibited at several Roboexotica events, serving drinks at the Vienniese Cocktail Robot Festival.
In 2014, two Robofriends were donated by Tom to us, and were further developed by Dr. Wummi and Karima Khlousy-Neirukh and other students at UAS Technikum Wien, in order to make Robofriend a playful companion robot which can be evaluated together with kids with special needs in selected schools in Vienna.
For this purpose, several game applications were developed which involve robot movements and expressions.


## Features
* live video transmission (low latency, < 300ms)
* remote control via webbrowser or game apps
* proximity sensors for collision avoidance & activity detection
* facial expressions, sound/voice playback, illuminated ears
* RFID reader - utilized for game interaction
* unique design (with CRT-head) by Tom Heike

## System Architecture / Components

The Robofriend uses a Teensy++ 2.0 microcontroller to handle motor control, led lights, servos, proximity sensors and battery status...
The Teensy is connected to the RaspberryPi via USB (/dev/ttyACM0). The RaspberryPi communicates with the Teensy via a simple protocol (ASCII-commands).
The RaspberryPi handles most of the robot's functions such as: RFID reader, CRT monitor, video camera, LED ears, speech output, ...

## User Interface and Control Options

The RaspberryPi starts a webserver which provides a webpage with live video feed and control interface.
It can be reached via http://<ip-adress-of-raspberry>:8765

The webpage uses an API to control the robots functions - this API is described in the theses of Karima Khlousy Neirukh (folder /documentation).
Furthermore, dedicated games have been developed by Karima which utilize the RFID-reader so that kids can use physical objects to interact with the robo and play litte games (identify animal sounds, emotions etc.)
These game applications are also written in python and are usually launched on a seperated tablet computer. The tablet connect to the python main script via a UDP connection (port 9000) and uses the same commands as the web GUI. It is planned to use a REST API for both (webpage and tablet GUI).

## Requirements

**Mjpg-Streamer:**
*  [MJPEG-Streamer Install & Setup](https://github.com/cncjs/cncjs/wiki/Setup-Guide:-Raspberry-Pi-%7C-MJPEG-Streamer-Install-&-Setup-&-FFMpeg-Recording)

**OpenCV 3:**
* To install OpenCV 3 on a Raspberry Pi 3 follow the following tutorial (except the virtual environment part):
[OpenCV 3 on Raspberry Pi 3](https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/)

**Roboter Operating System (ROS):**
* To install ROS on your computing unit (Laptop or Rasberry Pi 3) follow [these instructions](https://github.com/ProjectKitchen/RoboFriend/tree/ros-node-devel/src/Pi/scripts)

**LIDAR - 360 laser distance sensor lds-01:**
* Manufacturer's [site](http://www.robotis.us/360-laser-distance-sensor-lds-01-lidar/)
* Set permission for the LDS-01 sensor after connecting it to the USB interface:
   * sudo chmod a+rw /dev/ttyUSBx

## Additional Information

* We recommand you to clone the repository into a folder named *Git* placed in your home directory (~/Git/)
* After pulling the repository, execute following commands (see the ROS install instructions):
   * cd ~/Git/RoboFriend/src/Pi/catkin_ws/ && rm -r build/ && rm -r devel/ && catkin_make clean && catkin_make && catkin_make
* To start face and object detection on extrenal laptop enter *./start_computervision.sh <IP ADDRESS OF RASPBERRY PI>*

## Setup

Setup scripts are provided in folder /src/Pi/scripts

## Startup

Startup scripts are provided in folder /src/Pi/

## Future Plans

As next steps, several high-levels skills for Robofriend are planned:
* indoor localisation using ROS navigation stack (in order to find charging station)
* speech interaction (mic-array, beamforming, DOA, hotword detection)
* image analysis of live camera feed (face detection, facial landmark classification)
