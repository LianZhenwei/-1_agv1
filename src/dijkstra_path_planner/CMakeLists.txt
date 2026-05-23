cmake_minimum_required(VERSION 3.0.2)
project(dijkstra_path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  visualization_msgs
  by_global_path_planning
)

# 依赖Boost（目录遍历）
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf visualization_msgs by_global_path_planning
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 编译核心节点
add_executable(dijkstra_path_planner src/dijkstra_path_planner.cpp src/rviz.cpp)

add_dependencies(dijkstra_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接依赖库
target_link_libraries(dijkstra_path_planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 安装可执行文件
install(TARGETS dijkstra_path_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

