cmake_minimum_required(VERSION 2.8.3)
project(romansa)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(FILES rosa_dynamixel_msg.msg)


generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES romansa dynamixel serialport 
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamixel SHARED src/dynamixel.cpp)
add_library(serialport SHARED src/serialport.cpp)
#add_library(bulkread SHARED src/bulkread.cpp)
add_library(dynamixel_function SHARED src/dynamixel_function.cpp)

add_executable(rosa_dynamixel src/rosa_dynamixel.cpp)
add_dependencies(rosa_dynamixel rosa_dynamixel_generate_messages_cpp)
target_link_libraries(rosa_dynamixel ${catkin_LIBRARIES})
target_link_libraries(dynamixel_function dynamixel serialport)
target_link_libraries(rosa_dynamixel dynamixel serialport dynamixel_function)






