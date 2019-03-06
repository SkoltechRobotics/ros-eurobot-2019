cmake_minimum_required(VERSION 2.8.3)
project(eurobot_nav)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  tf2_ros
  tf2_msgs
  geometry_msgs
  OpenCV
)

generate_dynamic_reconfigure_options(
  config/Nav.cfg
)
find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package(
# CATKIN_DEPENDS dynamic_reconfigure rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_planner_node src/path_planner_node.cpp src/path_planner_node.h src/Astar.cpp src/Astar.h)
target_link_libraries(path_planner_node
 ${catkin_LIBRARIES}
)