cmake_minimum_required(VERSION 3.5)
project(tc_route_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tc_geo_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FollowerState.msg"
  "msg/ManagerStatus.msg"
  "msg/MissionInfo.msg"
  "msg/MotionLimits.msg"
  "msg/Route.msg"
  "msg/RouteState.msg"
  "msg/Waypoint.msg"
  "msg/ObstacleAvoidanceHint.msg"
  "msg/DriveModeStatus.msg"
  "msg/ActiveTargetLlh.msg"
  "msg/AvoidanceAdvice.msg"
  "srv/GetRoute.srv"
  "srv/UpdateRoute.srv"
  "srv/ReportStuck.srv"
  "action/RequestAvoidanceAdvice.action"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs builtin_interfaces tc_geo_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
