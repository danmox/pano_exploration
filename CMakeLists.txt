cmake_minimum_required(VERSION 2.8.3)
project(csqmi_exploration)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  csqmi_planning
  grid_mapping
  nodelet
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  rosbag
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    csqmi_planning
    grid_mapping
    nodelet
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    move_base_msgs
    rosbag
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(exploration_manager src/exploration_manager.cpp)
target_link_libraries(exploration_manager ${catkin_LIBRARIES} ${OpenCV_LIBS})
