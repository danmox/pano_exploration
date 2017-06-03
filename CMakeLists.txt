cmake_minimum_required(VERSION 2.8.3)
project(csqmi_planning)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  grid_mapping
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosbag grid_mapping
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(PROJECT_SOURCE_FILES
  src/common.cpp
  src/thinning.cpp
  src/partitioning.cpp
)

add_executable(skeleton src/main.cpp ${PROJECT_SOURCE_FILES})
target_link_libraries(skeleton ${catkin_LIBRARIES} ${OpenCV_LIBS})
