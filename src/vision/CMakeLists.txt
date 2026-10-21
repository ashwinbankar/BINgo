cmake_minimum_required(VERSION 3.20)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Build the executable
add_executable(webcam_creative_pub src/webcam_creative_pub.cpp)
target_include_directories(webcam_creative_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(webcam_creative_pub PUBLIC c_std_99 cxx_std_17)

# Link ROS 2 + OpenCV dependencies
target_link_libraries(webcam_creative_pub PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${OpenCV_LIBRARIES}
)

# Install the executable so ROS 2 can find it
install(TARGETS
  webcam_creative_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()