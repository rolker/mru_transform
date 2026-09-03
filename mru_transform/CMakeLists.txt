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
find_package(lifecycle_msgs REQUIRED)
find_package(mru_transform_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)

find_package(yaml-cpp REQUIRED)
# yaml-cpp 0.8 exports the namespaced target; older versions export a bare one.
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()


############################
## datum resolution core  ##
############################

# Pure, ROS-free polygon/param/VDatum precedence resolution. Shared by
# chart_datum_node and its unit test so the precedence is unit-testable.
add_library(datum_config SHARED src/datum_config.cpp)
target_include_directories(datum_config PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(datum_config ${YAML_CPP_TARGET})
# Export datum_config so downstream packages (e.g. cube_bathymetry's chart-prior
# importer, rolker/unh_marine_autonomy#163) can link resolve_datum/load_datum_config
# via find_package(mru_transform). Shares the package export set; header is installed
# by the install(DIRECTORY include/ ...) below. (#28)
install(TARGETS datum_config
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


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
  # nodes/ holds the four LifecycleNode classes, extracted so the in-process
  # lifecycle tests can construct them. They are not API: they sit in the
  # global namespace, and chart_datum_node.hpp includes proj.h, which this
  # package does not export -- so shipping them would offer downstream code a
  # header that cannot compile. Keep them out of the installed include tree.
  # (#34)
  PATTERN "nodes" EXCLUDE
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
  yaml-cpp
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
  ${lifecycle_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)
target_include_directories(sea_surface_estimator PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

install(TARGETS sea_surface_estimator
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  nav_sat_fix_to_velocity
  nodes/nav_sat_fix_to_velocity.cpp
)

target_include_directories(nav_sat_fix_to_velocity PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(nav_sat_fix_to_velocity
  ${lifecycle_msgs_TARGETS}
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


# tide_copier had no build target at all until #34, although
# launch/tide_copier_launch.py has always launched executable='tide_copier'.
add_executable(
  tide_copier
  nodes/tide_copier.cpp
)

target_link_libraries(tide_copier
  ${lifecycle_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${tf2_msgs_TARGETS}
)
target_include_directories(tide_copier PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

install(TARGETS tide_copier
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  chart_datum_node
  nodes/chart_datum_node.cpp
)

target_link_libraries(chart_datum_node
  ${lifecycle_msgs_TARGETS}
  datum_config
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  PkgConfig::PROJ
)

ament_target_dependencies(chart_datum_node
  geodesy
)

install(TARGETS chart_datum_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_twist_rotation test/test_twist_rotation.cpp)
  target_link_libraries(test_twist_rotation
    ${PROJECT_NAME}
    tf2::tf2
  )
  target_include_directories(test_twist_rotation PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  # Regression test for issue #23: subscribeCheck() subscribes once and stops.
  ament_add_gtest(test_subscribe_once test/test_subscribe_once.cpp)
  target_link_libraries(test_subscribe_once
    ${PROJECT_NAME}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
  )
  target_include_directories(test_subscribe_once PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  # Issue #25: datum resolution precedence + point-in-polygon + config parsing.
  ament_add_gtest(test_datum_config test/test_datum_config.cpp)
  target_link_libraries(test_datum_config
    datum_config
  )
  target_include_directories(test_datum_config PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  # The water-line lever arm sea_surface_estimator applies to odom Z before
  # calling it the sea surface (the node's original TODO, 2026-08-21).
  ament_add_gtest(test_water_line_offset test/test_water_line_offset.cpp)
  target_link_libraries(test_water_line_offset
    ${geometry_msgs_TARGETS}
    tf2::tf2
  )
  target_include_directories(test_water_line_offset PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  # Issue #32: the TF lookup behind the water-line correction -- argument
  # order (the sign convention), caching, invalidation and the failure paths --
  # exercised against a tf2_ros::Buffer populated with setTransform.
  ament_add_gtest(test_water_line_lever_arm test/test_water_line_lever_arm.cpp)
  target_link_libraries(test_water_line_lever_arm
    ${geometry_msgs_TARGETS}
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
  )
  target_include_directories(test_water_line_lever_arm PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  # Issue #34: configure -> cleanup -> configure must not throw
  # ParameterAlreadyDeclaredException, and must keep a parameter value the
  # operator set on the running node.
  # The default 60 s ctest TIMEOUT is too tight now: the live cases budget
  # roughly eight 15 s endpoint-discovery waits, and a two-case failure alone
  # has been measured at 30 s. Without the headroom a regression surfaces as an
  # opaque ctest timeout instead of a named gtest failure.
  ament_add_gtest(test_lifecycle_reconfigure test/test_lifecycle_reconfigure.cpp
    TIMEOUT 180)
  target_link_libraries(test_lifecycle_reconfigure
    ${lifecycle_msgs_TARGETS}
    datum_config
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${geographic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    PkgConfig::PROJ
  )
  ament_target_dependencies(test_lifecycle_reconfigure
    geodesy
  )
  target_include_directories(test_lifecycle_reconfigure PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  # The live pub/sub cases in this test prove negatives ("an inactive node
  # published nothing"), so unrelated ROS traffic on the machine could break
  # them -- or satisfy a subscription-count wait vacuously. Give the test its
  # own domain and keep discovery on this host; the test additionally namespaces
  # the nodes it creates itself, so a domain collision still cannot cross-talk.
  #
  # Scope of that claim, precisely: this protects against UNRELATED traffic. It
  # does NOT make two concurrent runs of THIS test independent -- the domain is
  # hardcoded and LOCALHOST discovery explicitly permits same-host peers, so two
  # agent worktrees running `colcon test` at the same time share both the domain
  # and the namespace, and a subscription-count wait can then be satisfied by
  # the other process. Nor does it apply at all when the gtest binary is run
  # directly, a normal debugging step; export the same variables by hand.
  # Discovery-server / static-peer settings inherited from the environment could
  # otherwise re-join the test to a wider graph regardless of the range, so they
  # are blanked here too.
  set_tests_properties(test_lifecycle_reconfigure PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=89;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_DISCOVERY_SERVER=;ROS_STATIC_PEERS=;FASTRTPS_DEFAULT_PROFILES_FILE=;CYCLONEDDS_URI="
  )

  # unh_echoboats_project11#339: strict priority-preference nav-source
  # arbitration (highest-priority fresh source wins, not the newest sample).
  ament_add_gtest(test_nav_source_selection test/test_nav_source_selection.cpp)
  target_link_libraries(test_nav_source_selection
    rclcpp::rclcpp
  )
  target_include_directories(test_nav_source_selection PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
endif()

ament_package()

