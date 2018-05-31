cmake_minimum_required(VERSION 2.8.3)
project(test_env)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    IK
    ckp_Matrix
  #CATKIN_DEPENDS
  #  roscpp
  #  std_msgs

  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(IK src/IK.cpp)
add_library(ckp_Matrix src/ckp_Matrix)
add_executable(IK_node src/IK_node.cpp)
target_link_libraries(IK_node IK ckp_Matrix ${catkin_LIBRARIES})
add_dependencies(IK_node armtest_generate_messages_cpp)


add_executable(encoder_test_node src/encoder_test_node.cpp)
target_link_libraries(encoder_test_node IK ${catkin_LIBRARIES})
add_dependencies(encoder_test_node armtest_generate_messages_cpp)
