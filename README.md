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

Following packages should be installed:
* mjpg-streamer: apt-get install -y gstreamer1.0-x gstreamer1.0-omx gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-alsa gstreamer1.0-libav
* espeak: apt-get install espeak
* python 2.7: apt-get install -y python python-pygame python-tk python-serial python-picamera python-pip
* python packages: pyttsx

## StartUp

Startup scripts are provided in folder /src/Pi/scripts

## Future Plans

As next steps, several high-levels skills for Robofriend are planned: 
* indoor localisation using IR / BLE-beacons (in order to find charging station)
* speech interaction (mic-array, beamforming, DOA, hotword detection)
* image analysis of live camera feed (face detection, facial landmark classification)
