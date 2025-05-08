cmake_minimum_required(VERSION 3.8)
project(usb_camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies(
  REQUIRED
  rclcpp
  rclcpp_components
  image_transport
  cv_bridge
  OpenCV
  sensor_msgs
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/usb_camera_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN usb_camera::USBCameraNode
  EXECUTABLE usb_camera_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)