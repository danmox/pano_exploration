cmake_minimum_required(VERSION 2.8.3)
project(panorama)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  nodelet
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
  sensor_msgs
  tf
  tf2_geometry_msgs
  tf2_kdl
  nav_msgs
  rosbag
  openni2_xtion
)

add_action_files(
  DIRECTORY
    action
  FILES
    Panorama.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS 
    cv_bridge
    geometry_msgs
    message_runtime 
    nodelet
    roscpp
    std_msgs
    actionlib_msgs
    actionlib
    sensor_msgs
    tf
    tf2_geometry_msgs
    tf2_kdl
    nav_msgs
    rosbag
    openni2_xtion
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENNI2_INCLUDE_DIR}
)

add_library(panorama_nodelet src/panorama_nodelet.cpp src/panorama.cpp)
add_dependencies(panorama_nodelet ${catkin_LIBRARIES} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(panorama_nodelet ${catkin_LIBRARIES} ${OPENNI2_LIBRARY})

add_executable(panorama_node src/panorama_node.cpp src/panorama.cpp)
add_dependencies(panorama_node ${catkin_LIBRARIES} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(panorama_node ${catkin_LIBRARIES} ${OPENNI2_LIBRARY})

add_executable(action_client src/panorama_action_client.cpp)
add_dependencies(action_client ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(action_client ${catkin_LIBRARIES})

add_executable(viz_pan src/show_panorama.cpp)
add_dependencies(viz_pan ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(viz_pan ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(panorama_cli src/panorama_cli.cpp)
add_dependencies(panorama_cli ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(panorama_cli ${catkin_LIBRARIES})
