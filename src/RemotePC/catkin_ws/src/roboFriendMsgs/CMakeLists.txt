cmake_minimum_required(VERSION 2.8.3)
project(roboFriendMsgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  FILES
  robofriendDrive.msg
  robofriendOdom.msg
  irCamData.msg
  irToken.msg
  chargingStationValues.msg
)

add_service_files(
  FILES
  driveInChargingStation.srv
  setGoal.srv
)

generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_runtime)
