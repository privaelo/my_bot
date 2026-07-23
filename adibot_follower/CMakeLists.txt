cmake_minimum_required(VERSION 3.8)
project(adibot_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(target_driver src/target_driver.cpp)
ament_target_dependencies(target_driver
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(map_loader src/map_loader.cpp)
ament_target_dependencies(map_loader rclcpp nav_msgs)

add_executable(global_planner src/global_planner.cpp)
ament_target_dependencies(global_planner
  rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros)

install(TARGETS target_driver map_loader global_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # no per-file copyright headers in this repo; LICENSE covers the package
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_a_star test/test_a_star.cpp)
  target_include_directories(test_a_star PRIVATE include)
endif()

ament_package()
