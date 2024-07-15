cmake_minimum_required(VERSION 3.12.0)

project(airsim_interfaces)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalAngleEulerCmd.msg"
  "msg/GimbalAngleQuatCmd.msg"
  "msg/GPSYaw.msg"
  "msg/VelCmd.msg"
  "msg/VelCmdGroup.msg"
  "msg/CarControls.msg"
  "msg/CarState.msg"
  "msg/ComputerVisionState.msg"
  "msg/InstanceSegmentationLabel.msg"
  "msg/InstanceSegmentationList.msg"
  "msg/ObjectTransformsList.msg"
  "msg/StringArray.msg"
  "msg/Altimeter.msg"
  "msg/Environment.msg"
  "srv/SetGPSPosition.srv"
  "srv/Takeoff.srv"
  "srv/TakeoffGroup.srv"
  "srv/Land.srv"
  "srv/LandGroup.srv"
  "srv/Reset.srv"
  "srv/RefreshInstanceSegmentation.srv"
  "srv/RefreshObjectTransforms.srv"
  "srv/SetLocalPosition.srv" DEPENDENCIES std_msgs geometry_msgs geographic_msgs LIBRARY_NAME ${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
