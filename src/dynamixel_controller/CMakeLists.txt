cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES romansa dynamixel serialport bulkread
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_controller src/dynamixel_controller.cpp)
target_link_libraries(dynamixel_controller ${catkin_LIBRARIES})
