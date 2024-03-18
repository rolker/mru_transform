cmake_minimum_required(VERSION 3.8)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
  project11
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  tf2_geometry_msgs
  mru_transform_interfaces
  )

set(EXTERNAL_DEPENDS

  )

set(EXTERNAL_DEPENDS_LIB

  )

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})  ## this package name is the name of the directory this cmake file is in

add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# Default to C++17 for humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
###  Add dependencies
###

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)

file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

include_directories(
  include
)

###########
## Build ##
###########

set(EXECUTABLES "")  # for installing later

message("buliding ${PROJECT_NAME} node: ")

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})
  list(APPEND EXECUTABLES ${NODE_NAME})
  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS})
  ament_target_dependencies(${NODE_NAME} ${ROS_DEPENDS} ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/" ${LIBSERIALPORT_INCLUDE_DIRS})
  target_link_libraries(${NODE_NAME} ${LIBSERIALPORT_LIBRARIES})
endforeach(NODE_FILE)

message("  Installing: " ${EXECUTABLES})
install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}/
)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)



#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



# cmake_minimum_required(VERSION 2.8.3)
# project(mru_transform)

# add_compile_options(-std=c++14)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   project11
#   roscpp
#   tf2
#   tf2_ros
#   message_generation
#   nav_msgs
#   sensor_msgs
#   geometry_msgs
#   geographic_msgs
# )

# ## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   LatLongToEarth.srv
#   LatLongToMap.srv
#   EarthToLatLong.srv
#   MapToLatLong.srv
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   geographic_msgs
# )

# catkin_package(
#     CATKIN_DEPENDS project11 roscpp tf2 tf2_ros
# )

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
# )

# set( SOURCES
#   src/map_frame.cpp
#   src/mru_transform.cpp
#   src/orientation_sensor.cpp
#   src/position_sensor.cpp
#   src/sensor.cpp
#   src/velocity_sensor.cpp
# )

# add_executable(${PROJECT_NAME}_node src/mru_transform_node.cpp ${SOURCES})
# add_executable(sensor_sync_node src/sensor_sync.cpp)

# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(sensor_sync_node
#   ${catkin_LIBRARIES}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
