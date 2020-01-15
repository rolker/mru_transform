cmake_minimum_required(VERSION 2.8.3)
project(project11_transformations)

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
  marine_msgs
  geodesy
  message_generation
  nav_msgs
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
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES project11_transformations
    CATKIN_DEPENDS project11 roscpp tf2 tf2_ros
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/broadcaster.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_node marine_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
