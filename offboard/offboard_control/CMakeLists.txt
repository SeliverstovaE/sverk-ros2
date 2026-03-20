cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(offboard_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)


include_directories(include)

add_executable(offboard_control src/offboard_control.cpp)
target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

# Установка C++17 стандарта
target_compile_features(offboard_control PUBLIC cxx_std_17)

ament_target_dependencies(offboard_control
 rclcpp 
 px4_msgs
 px4_ros_com
 std_srvs
 offboard_interfaces
 std_msgs
 geometry_msgs

 tf2_ros
 tf2_geometry_msgs
)

# В target_link_libraries добавить:
# target_link_libraries(${PROJECT_NAME}
#   ${Eigen3_LIBRARIES}
#   tf2::tf2
# )

install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()