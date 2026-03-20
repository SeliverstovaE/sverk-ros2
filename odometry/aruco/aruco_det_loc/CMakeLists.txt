cmake_minimum_required(VERSION 3.8)
project(aruco_det_loc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

# ----- Messages -----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ----- Include dirs (for headers like aruco_det_loc/aruco_loc_utils.hpp) -----
include_directories(include)

# ----- Detect Node -----
add_executable(aruco_detect_node src/aruco_detect_node.cpp)

ament_target_dependencies(aruco_detect_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

rosidl_target_interfaces(aruco_detect_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS aruco_detect_node
  DESTINATION lib/${PROJECT_NAME}
)

# ----- Localization Node -----
add_executable(aruco_loc_node src/aruco_loc_node.cpp)

ament_target_dependencies(aruco_loc_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

rosidl_target_interfaces(aruco_loc_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS aruco_loc_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка launch файлов
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ----- Install headers -----
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
