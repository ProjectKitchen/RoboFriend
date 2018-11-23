# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robofriend: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irobofriend:/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robofriend_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg" NAME_WE)
add_custom_target(_robofriend_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robofriend" "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robofriend
  "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robofriend
)

### Generating Services

### Generating Module File
_generate_module_cpp(robofriend
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robofriend
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robofriend_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robofriend_generate_messages robofriend_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg" NAME_WE)
add_dependencies(robofriend_generate_messages_cpp _robofriend_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robofriend_gencpp)
add_dependencies(robofriend_gencpp robofriend_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robofriend_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robofriend
  "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robofriend
)

### Generating Services

### Generating Module File
_generate_module_eus(robofriend
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robofriend
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robofriend_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robofriend_generate_messages robofriend_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg" NAME_WE)
add_dependencies(robofriend_generate_messages_eus _robofriend_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robofriend_geneus)
add_dependencies(robofriend_geneus robofriend_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robofriend_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robofriend
  "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robofriend
)

### Generating Services

### Generating Module File
_generate_module_lisp(robofriend
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robofriend
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robofriend_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robofriend_generate_messages robofriend_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg" NAME_WE)
add_dependencies(robofriend_generate_messages_lisp _robofriend_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robofriend_genlisp)
add_dependencies(robofriend_genlisp robofriend_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robofriend_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robofriend
  "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robofriend
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robofriend
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robofriend
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robofriend_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robofriend_generate_messages robofriend_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg" NAME_WE)
add_dependencies(robofriend_generate_messages_nodejs _robofriend_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robofriend_gennodejs)
add_dependencies(robofriend_gennodejs robofriend_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robofriend_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robofriend
  "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robofriend
)

### Generating Services

### Generating Module File
_generate_module_py(robofriend
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robofriend
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robofriend_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robofriend_generate_messages robofriend_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/project/RoboFriend/src/Pi/catkin_ws/src/robofriend/msg/Coordinates.msg" NAME_WE)
add_dependencies(robofriend_generate_messages_py _robofriend_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robofriend_genpy)
add_dependencies(robofriend_genpy robofriend_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robofriend_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robofriend)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robofriend
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robofriend_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robofriend)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robofriend
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robofriend_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robofriend)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robofriend
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robofriend_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robofriend)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robofriend
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robofriend_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robofriend)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robofriend\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robofriend
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robofriend_generate_messages_py std_msgs_generate_messages_py)
endif()
