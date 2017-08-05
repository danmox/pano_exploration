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
  LIBRARIES ${PROJECT_NAME}
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
  src/csqmi.cpp
)

add_library(${PROJECT_NAME} ${PROJECT_SOURCE_FILES})
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(planning src/main.cpp ${PROJECT_SOURCE_FILES})
target_link_libraries(planning ${catkin_LIBRARIES} ${OpenCV_LIBS})
