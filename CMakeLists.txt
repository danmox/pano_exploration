cmake_minimum_required(VERSION 2.8.3)
project(panorama)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
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
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(panorama_SOURCE_FILES
    src/panorama_nodelet.cpp
    src/panorama.cpp
    src/panorama_node.cpp
)

add_library(panorama_nodelet ${panorama_SOURCE_FILES})
add_dependencies(panorama_nodelet ${catkin_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(panorama_nodelet ${catkin_LIBRARIES})

add_executable(panorama_node ${panorama_SOURCE_FILES})
add_dependencies(panorama_node ${catkin_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(panorama_node ${catkin_LIBRARIES})

add_executable(action_client src/panorama_action_client.cpp)
add_dependencies(action_client ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(action_client ${catkin_LIBRARIES})
