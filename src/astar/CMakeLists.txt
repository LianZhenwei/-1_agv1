########### 版本要求
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

########### 项目名称
project(astar)

########### 添加系统库
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        by_global_path_planning
        )

catkin_package()

########### 添加头文件
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
add_executable(${PROJECT_NAME} src/main.cpp src/Astar.cpp src/OccMapTransform.cpp src/BSpline.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})