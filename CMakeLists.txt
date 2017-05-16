cmake_minimum_required(VERSION 2.8.3)
project(grid_mapping)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  rosbag
  geometry_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    nav_msgs
    sensor_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/angle_grid.cpp
  src/grid_base.cpp
  src/occ_grid.cpp
  src/point.cpp
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(grid_test src/main.cpp src/common.cpp)
target_link_libraries(grid_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
