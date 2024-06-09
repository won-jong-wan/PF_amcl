cmake_minimum_required(VERSION 3.5)
project(PF_amcl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
#include_directories(/opt/ros/humble/include/tf2)
#include_directories(/opt/ros/humble/include/tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add executable the generate | name must be the same as the launch file
add_executable(PF_node src/potentialF.cpp)
# Link dependencies to the executable | name must be the same as the launch file
ament_target_dependencies(potential_field_node rclcpp nav_msgs sensor_msgs tf2_geometry_msgs tf2 tf2_ros)

# install the executable
install(TARGETS
PF_node
  DESTINATION lib/${PROJECT_NAME})
# install the launch executable
install(DIRECTORY
  launch map param rviz worlds
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
