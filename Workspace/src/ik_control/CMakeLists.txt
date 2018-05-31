cmake_minimum_required(VERSION 2.8.3)
project(ik_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  can_msgs
  genmsg message_generation
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs can_msgs)

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

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node test_node_generate_messages_cpp)

add_executable(arm_controller_node src/arm_controller_node.cpp)
target_link_libraries(arm_controller_node ${catkin_LIBRARIES})
add_dependencies(arm_controller_node arm_controller_node_generate_messages_cpp)

add_executable(arm_model_node src/arm_model_node.cpp)
target_link_libraries(arm_model_node ${catkin_LIBRARIES})
add_dependencies(arm_model_node arm_model_node_generate_messages_cpp)

add_executable(arm_controller_node_ML src/arm_controller_node_ML.cpp)
target_link_libraries(arm_controller_node_ML ${catkin_LIBRARIES})
add_dependencies(arm_controller_node_ML arm_controller_node_ML_generate_messages_cpp)

add_executable(arm_controller_node_ML_CAN src/arm_controller_node_ML_CAN.cpp)
target_link_libraries(arm_controller_node_ML_CAN ${catkin_LIBRARIES})
add_dependencies(arm_controller_node_ML_CAN arm_controller_node_ML_CAN_generate_messages_cpp)

