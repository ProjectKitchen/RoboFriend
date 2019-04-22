## Installing Ubuntu

**Ubuntu Xenial (16.04 LTS):**
* [Download](http://releases.ubuntu.com/16.04/ubuntu-16.04.5-desktop-amd64.iso) and install the desktop image for 64-bit PC (AMD64) computers
* [Download](http://releases.ubuntu.com/16.04/ubuntu-16.04.5-desktop-i386.iso) and install the desktop image for 32-bit PC (i386) computers

**Ubuntu Xenial (16.04 LTS) and ROS Kinetic pre-installed for Raspberry Pi 3:**
* Download the latest image from this [Website](https://downloads.ubiquityrobotics.com/pi.html) and read the notes.

## Installing ROS
* There are different ways to set up ROS on your computing unit

* To manually install ROS on a Ubuntu platform, visit the [official ROS wiki](http://wiki.ros.org/ROS/Installation). There are two main releases at the current time:
    * ROS Kinetic is available for Ubuntu Xenial (16.04 LTS) and Debian Jessie, among other platform options.
    * ROS Melodic is avialable for Ubuntu Bionic (18.04 LTS) and Debian Stretch, among other platform options.

It is strongly recommended to use [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Follow the next heading for a simplified installation.

## Setup project
	$ mkdir ~/Git && cd ~/Git/
	$ sudo apt-get update
	$ sudo apt-get intsall git
	$ git clone https://github.com/ProjectKitchen/RoboFriend
	$ cd ~/Git/Robofriend/
	$ (checkout the needed branch)
	$ cd ~/Git/Robofriend/src/Pi/scripts/
	$ ./autostart.sh (only on Raspberry Pi)
	$ ./install_ros_kinetic.sh
	$ ./install_python.modules.sh
	$ ./install_ros_pkg.sh

## Robofriend startup scripts

**autostart.sh**
* puts startrobo.desktop into the folder ~/.config/autostart
* puts startrobo.sh into folder ~/
* copies rc.local into the folder /etc/

**install_python_modules.sh**
* installs all necessary python packages

**install_ros_kinetic.sh**
* installs ROS Kinetic on remote PC

**install_ros_pkg.sh**
* installs required ROS packages

**install_user_modules.sh**
* installs software packages for further development

**startrobo.sh**
* starts the videostream and the main python script after system boot.

## Debugging and other notes

To see the command line output during the operation of the main python script: 
* log into the rasperry pi from another computer (using e.g. putty or ssh)
* sudo pkill python (to stop the currently running python scripts)
* run ./startrobo.sh placed in your home folder

Your user should have sudo rights. To use sudo without password add the following line to /etc/sudoers
pi ALL=(ALL) NOPASSWD: ALL

(Be aware that using sudo without password is not a recommended practise because it compromises security on your system)
