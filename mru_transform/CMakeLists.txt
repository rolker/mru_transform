cmake_minimum_required(VERSION 3.14.4)
project(mru_transform)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geodesy REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mru_transform_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)


#############
## library ##
#############


add_library(${PROJECT_NAME} SHARED
  src/map_frame.cpp
  src/mru_transform.cpp
  src/navigation_sensors.cpp
  src/orientation_sensor.cpp
  src/position_sensor.cpp
  src/velocity_sensor.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}
  ${geographic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mru_transform_interfaces_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
)

ament_target_dependencies(${PROJECT_NAME}
  geodesy
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geodesy
  geographic_msgs
  geometry_msgs
  mru_transform_interfaces
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

#############
##  nodes  ##
#############

add_executable(
  mru_transform_node
  nodes/mru_transform_node.cpp
)

target_link_libraries(mru_transform_node
  ${PROJECT_NAME}
)

install(TARGETS mru_transform_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  sea_surface_estimator
  nodes/sea_surface_estimator.cpp
)

target_link_libraries(sea_surface_estimator
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
)

install(TARGETS sea_surface_estimator
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  nav_sat_fix_to_velocity
  nodes/nav_sat_fix_to_velocity.cpp
)

target_link_libraries(nav_sat_fix_to_velocity
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geographic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

ament_target_dependencies(nav_sat_fix_to_velocity
  geodesy
)

install(TARGETS nav_sat_fix_to_velocity
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  chart_datum_node
  nodes/chart_datum_node.cpp
)

target_link_libraries(chart_datum_node
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  PkgConfig::PROJ
)

install(TARGETS chart_datum_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

