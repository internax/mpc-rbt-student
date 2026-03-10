cmake_minimum_required(VERSION 3.5)
project(cv_3_pkg)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cv_3_node src/cv_3_node.cpp)
ament_target_dependencies(cv_3_node rclcpp std_msgs)

install(TARGETS
  cv_3_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
