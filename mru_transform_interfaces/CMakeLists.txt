cmake_minimum_required(VERSION 3.8)

project(mru_transform_interfaces)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  srv/EarthToLatLong.srv
  srv/LatLongToEarth.srv
  srv/LatLongToMap.srv
  srv/MapToLatLong.srv)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    ${MSG_FILES}
    DEPENDENCIES
    builtin_interfaces
    geographic_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()


# cmake_minimum_required(VERSION 3.8)

# project(mru_transform_interfaces)

# find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/EarthToLatLong.srv"
#   "srv/LatLongToEarth.srv"
#   "srv/LatLongToMap.srv
#   "srv/MapToLatLong.srv"
#   DEPENDENCIES
#   std_msgs)

# ament_export_dependencies(rosidl_default_runtime)

# ament_package()
