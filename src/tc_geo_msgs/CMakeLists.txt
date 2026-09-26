cmake_minimum_required(VERSION 3.5)
project(tc_geo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FusionState.msg"
  "msg/GeoPoint.msg"
  "msg/GeoPose.msg"
  "msg/GeoPoseWithQuality.msg"
  "msg/MapProjection.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
