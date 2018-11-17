


# Robofriend Raspberry Pi implementation
# Web Controlled Robot With Video Stream

 credits: Tom Heike for original Robofriend and mechanical platform
          Karima Khlousy Neirukh and Florian Hackl (Dr. Wummi) for HW-updates and Software rework
          Chris Veigl for coordination and glue logic


- robofriend.py is the main python script (currently Python2.7 is used)
- it handles UDP communication for remote control, keyboard input and opens a Flask Webserver for webbased remote control (port: 8765)
- in the background, MJPG-Streamer is running and transmits a live camera image to the webpage (see start_robofriend.sh)
- the motor control and collision prevention is done with a Teensy2.0++ microcontroller which is connected via USB (TTYACM0)
- an RFID reader is connected to USB (TTYUSB0)
- an iowarrior board is connected via USB (HID) - it allows to switch on/off the 12V power supply for the CRT TV and to control the LED-ears (PWM) and camera servo

- Raspbian Image - Raspbian Strech with Desktop: https://www.raspberrypi.org/downloads/raspbian/
- Etcher - Flash Software: https://www.balena.io/etcher/