# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/project/RoboFriend/src/Pi/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/project/RoboFriend/src/Pi/catkin_ws/build

# Utility rule file for robofriend_generate_messages_cpp.

# Include the progress variables for this target.
include robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/progress.make

robofriend/CMakeFiles/robofriend_generate_messages_cpp: /home/pi/project/RoboFriend/src/Pi/catkin_ws/devel/include/robofriend/Coordinates.h


/home/pi/project/RoboFriend/src/Pi/catkin_ws/devel/include/robofriend/Coordinates.h: /opt/ros/kinetic/lib/gencpp/gen_cpp.py
/home/pi/project/RoboFriend/src/Pi/catkin_ws/devel/include/robofriend/Coordinates.h: /home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg
/home/pi/project/RoboFriend/src/Pi/catkin_ws/devel/include/robofriend/Coordinates.h: /opt/ros/kinetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/project/RoboFriend/src/Pi/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from robofriend/Coordinates.msg"
	cd /home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend && /home/pi/project/RoboFriend/src/Pi/catkin_ws/build/catkin_generated/env_cached.sh /usr/bin/python /opt/ros/kinetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg -Irobofriend:/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg -Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg -p robofriend -o /home/pi/project/RoboFriend/src/Pi/catkin_ws/devel/include/robofriend -e /opt/ros/kinetic/share/gencpp/cmake/..

robofriend_generate_messages_cpp: robofriend/CMakeFiles/robofriend_generate_messages_cpp
robofriend_generate_messages_cpp: /home/pi/project/RoboFriend/src/Pi/catkin_ws/devel/include/robofriend/Coordinates.h
robofriend_generate_messages_cpp: robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/build.make

.PHONY : robofriend_generate_messages_cpp

# Rule to build all files generated by this target.
robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/build: robofriend_generate_messages_cpp

.PHONY : robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/build

robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/clean:
	cd /home/pi/project/RoboFriend/src/Pi/catkin_ws/build/robofriend && $(CMAKE_COMMAND) -P CMakeFiles/robofriend_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/clean

robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/depend:
	cd /home/pi/project/RoboFriend/src/Pi/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/project/RoboFriend/src/Pi/catkin_ws/src /home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend /home/pi/project/RoboFriend/src/Pi/catkin_ws/build /home/pi/project/RoboFriend/src/Pi/catkin_ws/build/robofriend /home/pi/project/RoboFriend/src/Pi/catkin_ws/build/robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robofriend/CMakeFiles/robofriend_generate_messages_cpp.dir/depend
