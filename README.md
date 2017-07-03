# Robofriend

Robofriend - a semi-autonomous, remotely controllable robot companion.

Robofriend - initially created by Tom Heike 2002 - was exhibited at several Roboexotica events, serving drinks at the Vienniese Cocktail Robot Festival.
In 2014, two Robofriends were donated by Tom to us, and were further developed by Dr. Wummi and Karima Khlousy-Neirukh.


Robofriend shall be used to evaluate playful interaction with kids with cognitive disabilites in special schools in Vienna.
For this purpose, several game applications were developed which involve robot movements and expressions.

The Robofriend platform was based on a VIA Epia single board PC and a C-Control microcontroller, 
which have been replaced by a RaspberryPi-2 and a Teensy microcontroller as low-level interface.

## Features
* live video transmission (low latency) 
* remote control via webbrowser or game apps
* proximity sensors for collision avoidance & activity detection
* facial expressions, sound/voice playback
* RFID reader - utilized for game interaction
* unique design (with CRT-head) by Tom Heike
 
## Setup

 The Robofriend uses a Teensy 2.0 ++ which handels motor driving, proximity sensors, battery status... you can find all needed source files in src/Teensy (readme included)

The Teensy is connected via USB to a RasPi 2 which handles most of the robot's functions such as: RFID reader, CRT monitor, RaspiCam, LED ears... It also includes a webserver, on which you can see the output of the RaspiCam (robot's view)... you can find the source files in src/Pi (readme included)

As UI for the children a normal (windows) tablet is used. It is absolutely necessary since the developed games and ideas wouldn't function without it (also used as interaction). For the source files please look into src/TabletGUI (readme included)