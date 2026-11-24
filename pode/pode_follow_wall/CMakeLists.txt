cmake_minimum_required(VERSION 3.8)
project(pode_follow_wall)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 진단 메시지
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/FollowWallState.msg
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# 계산부(노드 없이 시험)
add_library(pode_follow_wall_geometry SHARED src/wall_geometry.cpp)
target_include_directories(pode_follow_wall_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(pode_follow_wall_geometry PUBLIC
  ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS})

# controller 플러그인
add_library(pode_follow_wall_plugin SHARED src/follow_wall.cpp)
target_link_libraries(pode_follow_wall_plugin PUBLIC
  pode_follow_wall_geometry
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core
  tf2::tf2
  tf2_ros::tf2_ros
  ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS})

target_include_directories(pode_follow_wall_plugin PUBLIC ${nav2_core_INCLUDE_DIRS})

pluginlib_export_plugin_description_file(nav2_core follow_wall_plugin.xml)

install(TARGETS pode_follow_wall_geometry pode_follow_wall_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES follow_wall_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
  ament_add_gtest(test_wall_geometry test/test_wall_geometry.cpp)
  target_link_libraries(test_wall_geometry pode_follow_wall_geometry)
  ament_add_gtest(test_follow_wall test/test_follow_wall.cpp TIMEOUT 120)
  target_link_libraries(test_follow_wall pode_follow_wall_plugin)
  ament_target_dependencies(test_follow_wall nav2_regulated_pure_pursuit_controller)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
