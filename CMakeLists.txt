cmake_minimum_required(VERSION 3.0.2)
project(amazon_hackon)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(platform include/platform.h src/platform.cpp)
target_link_libraries(platform ${catkin_LIBRARIES})

