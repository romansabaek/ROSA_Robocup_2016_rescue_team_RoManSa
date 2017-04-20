cmake_minimum_required(VERSION 2.8.3)
project(rosa_msg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(FILES command.msg)
add_message_files(FILES readdata.msg)
add_message_files(FILES kinematics.msg)

generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosa_msg
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


