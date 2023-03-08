cmake_minimum_required(VERSION 2.8.3)
project(mru_transform)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  project11
  roscpp
  tf2
  tf2_ros
  message_generation
  nav_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LatLongToEarth.srv
  LatLongToMap.srv
  EarthToLatLong.srv
  MapToLatLong.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
  geographic_msgs
)

catkin_package(
    CATKIN_DEPENDS project11 roscpp tf2 tf2_ros
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set( SOURCES
  src/map_frame.cpp
  src/mru_transform.cpp 
  src/orientation_sensor.cpp
  src/position_sensor.cpp
  src/velocity_sensor.cpp
)

add_executable(${PROJECT_NAME}_node src/mru_transform_node.cpp ${SOURCES})
add_executable(sensor_sync_node src/sensor_sync.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(sensor_sync_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
