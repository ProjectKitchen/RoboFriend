## Robofriend startup scripts

these scipts start the videostream and the main python script automatically after system boot.

put robofriend.desktop into folder /home/pi/.config/autostart
put startrobo.sh into folder /home/pi

to see commandline output (debug messages) during operation of main python script you could: 
* log into the rasperry pi from another computer (using e.g. putty or ssh)
* sudo pkill python (to stop the currently running python scripts)
* run ./startrobo.sh in folder /home/pi

your user should have sudo rights to do that
to use sudo without password add the following line to /etc/sudoers
pi ALL=(ALL) NOPASSWD: ALL

(be aware that using sudo without password is not a recommended practise because it compromises security on your system)

