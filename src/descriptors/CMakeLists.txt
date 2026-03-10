cmake_minimum_required(VERSION 3.8)
project(descriptors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster rclcpp tf2_ros geometry_msgs nav_msgs)

install(
  DIRECTORY urdf launch rviz sdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
