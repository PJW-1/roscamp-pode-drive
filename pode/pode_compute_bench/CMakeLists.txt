cmake_minimum_required(VERSION 3.8)
project(pode_compute_bench)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pode_rl_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(compute_bench src/compute_bench.cpp)
ament_target_dependencies(compute_bench
  geometry_msgs nav2_core nav2_costmap_2d nav2_map_server nav2_util nav_msgs pluginlib
  pode_rl_controller rclcpp rclcpp_lifecycle sensor_msgs tf2 tf2_ros)

install(TARGETS compute_bench DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/bench_table.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bench_table test/test_bench_table.py)
  ament_add_pytest_test(test_cases test/test_cases.py)
endif()

ament_package()
