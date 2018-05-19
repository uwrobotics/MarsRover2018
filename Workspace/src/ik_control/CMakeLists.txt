cmake_minimum_required(VERSION 2.8.3)
project(ik_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Declare a catkin package
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



###########
## Build ##
###########

add_executable(armtest src/armtest.cpp)
target_link_libraries(armtest ${catkin_LIBRARIES})
add_dependencies(armtest armtest_generate_messages_cpp)




