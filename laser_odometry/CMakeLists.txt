cmake_minimum_required(VERSION 3.8)
project(laser_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(PkgConfig)
find_package(csm REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(plicp_odometry src/plicp_odometry.cpp)
ament_target_dependencies(plicp_odometry 
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  csm
  tf2_ros
  tf2_eigen
)
target_include_directories(plicp_odometry
  PRIVATE
  "include"
)

ament_target_dependencies(plicp_odometry
  rclcpp
  sensor_msgs
  rcutils
  Eigen3
  tf2_ros
  tf2
  tf2_geometry_msgs
  csm
  Boost
  nav_msgs
)

install(TARGETS
  plicp_odometry
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
