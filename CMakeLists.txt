cmake_minimum_required(VERSION 2.8.3)
project(laser_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roslaunch
  roscpp
  rospy
  std_msgs
  message_generation
)


 add_message_files(  
   FILES
   coor.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_slam
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
  ./include
  ${catkin_INCLUDE_DIRS}
  ~/serial/include
)

link_directories("~/serial/build/devel/lib" include)

add_executable(coor_receiver src/coor_receiver.cpp)
target_link_libraries(coor_receiver libserial.so ${catkin_LIBRARIES})


add_executable(motor_control src/motor_control.cpp src/action_driver.cpp)
target_link_libraries(motor_control ${catkin_LIBRARIES})




