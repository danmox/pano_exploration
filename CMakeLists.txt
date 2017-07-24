cmake_minimum_required(VERSION 2.8.3)
project(grid_mapping)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  nodelet
  rosbag
  tf
)

find_package(OpenCV REQUIRED)

add_message_files(FILES
  OccupancyGrid.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    nodelet
    rosbag
    tf
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
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(angle_grid_nodelet src/angle_grid_nodelet.cpp)
add_dependencies(angle_grid_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(angle_grid_nodelet ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(grid_test src/main.cpp src/common.cpp)
add_dependencies(grid_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(grid_test ${PROJECT_NAME} ${catkin_LIBRARIES} 
  ${OpenCV_LIBS})
