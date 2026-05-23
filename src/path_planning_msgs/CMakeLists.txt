cmake_minimum_required(VERSION 2.8.3)
project(path_planning_msgs)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
message_generation
std_msgs
)
add_message_files(
  FILES
  Path.msg
  PathPoint.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs
  message_runtime
  std_msgs 
)

