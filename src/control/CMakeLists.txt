cmake_minimum_required(VERSION 3.20)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dummy_twist_pub src/dummy_twist_pub.cpp)
target_link_libraries(dummy_twist_pub rclcpp::rclcpp geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)

install(TARGETS
  dummy_twist_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()