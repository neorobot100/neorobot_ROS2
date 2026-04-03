cmake_minimum_required(VERSION 3.8)
project(neo_robot_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Dependencies =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# ===== Motion Arbiter =====
add_executable(motion_arbiter src/motion_arbiter.cpp)
ament_target_dependencies(motion_arbiter
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  rclcpp_action
  nav2_msgs
)

# ===== lidar_human_detect =====
add_executable(lidar_human_detect src/lidar_human_detect.cpp)
ament_target_dependencies(lidar_human_detect
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  rclcpp_action
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
)


# ===== lidar_human_leg detect =====
add_executable(lidar_human_leg_detect src/lidar_human_leg_detect.cpp)
ament_target_dependencies(lidar_human_leg_detect
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  rclcpp_action
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# ===== human and obstacle detect =====
add_library(human_n_obstacle_layer SHARED
  src/human_n_obstacle_layer.cpp
)
ament_target_dependencies(human_n_obstacle_layer
  rclcpp
  geometry_msgs
  std_msgs
  # sensor_msgs
  # rclcpp_action
  # nav2_msgs
  nav2_costmap_2d
  pluginlib
  tf2_geometry_msgs   
)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)


# ===== Install =====
install(TARGETS
  motion_arbiter
  lidar_human_leg_detect
  DESTINATION lib/${PROJECT_NAME}
)

# plugin library
install(TARGETS human_n_obstacle_layer
  LIBRARY DESTINATION lib
)
ament_package()