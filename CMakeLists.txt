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
  scarab_msgs
  rosbag
  panorama
  pcl_ros
  pcl_conversions
  sensor_msgs
  message_generation
  relative_localization
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
    PanGoal.msg
)

add_service_files(
  FILES
    InitRelLocalization.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

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
    sensor_msgs
    nav_msgs
    scarab_msgs
    rosbag
    panorama
    pcl_ros
    pcl_conversions
    message_runtime
    relative_localization
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(exploration_manager src/exploration_manager.cpp)
target_link_libraries(exploration_manager ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(exploration_manager ${PROJECT_NAME}_generate_messages_cpp)

add_executable(skel_pointcloud src/pcl_visualization_test.cpp)
target_link_libraries(skel_pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(init_relative_localization src/init_relative_localization.cpp)
target_link_libraries(init_relative_localization ${catkin_LIBRARIES})
add_dependencies(init_relative_localization ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
