cmake_minimum_required(VERSION 3.8)
project(hb_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

#COMPILE
add_executable(
  hb_node  src/hb_ctrl.cpp
)

target_link_libraries(hb_node pigpiod_if2)

ament_target_dependencies(
  hb_node
  rclcpp
  std_msgs
  sensor_msgs
)

# INSTALL
install(
  TARGETS
  hb_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
