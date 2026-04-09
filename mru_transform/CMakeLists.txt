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
  ${geometry_msgs_TARGETS}
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

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

#####################
## VDatum grids    ##
#####################

# Cache directory for downloaded grids (persists across builds)
set(VDATUM_CACHE_DIR "$ENV{HOME}/.cache/mru_transform" CACHE PATH
  "Cache directory for downloaded VDatum and PROJ grids")

set(GEOID_GRID "us_noaa_g2018u0.tif")
set(GEOID_CACHE "${VDATUM_CACHE_DIR}/geoid/${GEOID_GRID}")
set(VDATUM_ZIP_URL "https://vdatum.noaa.gov/download/data/vdatum_regional_20250917.zip")
set(VDATUM_ZIP_CACHE "${VDATUM_CACHE_DIR}/vdatum_regional.zip")
set(VDATUM_EXTRACT_DIR "${VDATUM_CACHE_DIR}/vdatum")

# Download PROJ geoid grid if not cached
if(NOT EXISTS "${GEOID_CACHE}")
  message(STATUS "Downloading PROJ geoid grid (${GEOID_GRID})...")
  file(MAKE_DIRECTORY "${VDATUM_CACHE_DIR}/geoid")
  execute_process(
    COMMAND projsync --file "${GEOID_GRID}" --target-dir "${VDATUM_CACHE_DIR}/geoid"
    RESULT_VARIABLE GEOID_RESULT
  )
  if(NOT GEOID_RESULT EQUAL 0)
    message(WARNING "Failed to download PROJ geoid grid. "
      "chart_datum_node will not work without it. "
      "Run: projsync --file ${GEOID_GRID} --target-dir ${VDATUM_CACHE_DIR}/geoid")
  endif()
else()
  message(STATUS "PROJ geoid grid cached: ${GEOID_CACHE}")
endif()

# Download VDatum grids if not cached
file(GLOB MLLW_GRIDS "${VDATUM_EXTRACT_DIR}/vdatum/*/*_mllw.gtx")
list(LENGTH MLLW_GRIDS MLLW_COUNT)
if(MLLW_COUNT EQUAL 0)
  if(NOT EXISTS "${VDATUM_ZIP_CACHE}")
    message(STATUS "Downloading VDatum regional grids (~849MB)...")
    file(MAKE_DIRECTORY "${VDATUM_CACHE_DIR}")
    file(DOWNLOAD "${VDATUM_ZIP_URL}" "${VDATUM_ZIP_CACHE}"
      STATUS VDATUM_DL_STATUS
      SHOW_PROGRESS
    )
    list(GET VDATUM_DL_STATUS 0 VDATUM_DL_CODE)
    if(NOT VDATUM_DL_CODE EQUAL 0)
      message(WARNING "Failed to download VDatum grids. "
        "chart_datum_node will not work without them. "
        "Download manually: ${VDATUM_ZIP_URL}")
      set(VDATUM_ZIP_CACHE "")
    endif()
  else()
    message(STATUS "VDatum zip cached: ${VDATUM_ZIP_CACHE}")
  endif()

  if(EXISTS "${VDATUM_ZIP_CACHE}")
    message(STATUS "Extracting MLLW grids from VDatum archive...")
    file(MAKE_DIRECTORY "${VDATUM_EXTRACT_DIR}")
    execute_process(
      COMMAND unzip -o "${VDATUM_ZIP_CACHE}" "*_mllw.gtx" "*.bnd" "*.met"
      WORKING_DIRECTORY "${VDATUM_EXTRACT_DIR}"
      RESULT_VARIABLE UNZIP_RESULT
    )
    if(NOT UNZIP_RESULT EQUAL 0)
      message(WARNING "Failed to extract VDatum grids from ${VDATUM_ZIP_CACHE}")
    endif()
  endif()
else()
  message(STATUS "VDatum MLLW grids cached: ${MLLW_COUNT} files")
endif()

# Install grids to package share directory
if(EXISTS "${GEOID_CACHE}")
  install(FILES "${GEOID_CACHE}"
    DESTINATION share/${PROJECT_NAME}/data/geoid
  )
endif()

file(GLOB MLLW_GRIDS "${VDATUM_EXTRACT_DIR}/vdatum/*/*_mllw.gtx")
file(GLOB VDATUM_BND "${VDATUM_EXTRACT_DIR}/vdatum/*/*.bnd")
file(GLOB VDATUM_MET "${VDATUM_EXTRACT_DIR}/vdatum/*/*.met")
foreach(GRID_FILE ${MLLW_GRIDS} ${VDATUM_BND} ${VDATUM_MET})
  # Preserve the regional subdirectory structure
  file(RELATIVE_PATH REL_PATH "${VDATUM_EXTRACT_DIR}" "${GRID_FILE}")
  get_filename_component(REL_DIR "${REL_PATH}" DIRECTORY)
  install(FILES "${GRID_FILE}"
    DESTINATION share/${PROJECT_NAME}/data/${REL_DIR}
  )
endforeach()

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

