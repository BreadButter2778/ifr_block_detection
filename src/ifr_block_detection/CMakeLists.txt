cmake_minimum_required(VERSION 3.8)
project(ifr_block_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ifr_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# ---------- node 1: fake camera, publish /camera/compressed ----------
add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher
  rclcpp
  sensor_msgs
  ament_index_cpp)
target_link_libraries(image_publisher ${OpenCV_LIBS})
target_include_directories(image_publisher PRIVATE ${OpenCV_INCLUDE_DIRS})

# ---------- node 2: detector, subscribe image and publish results ----------
add_executable(block_detector src/block_detector.cpp)
ament_target_dependencies(block_detector
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  foxglove_msgs
  ifr_interfaces)
target_link_libraries(block_detector ${OpenCV_LIBS})
target_include_directories(block_detector PRIVATE ${OpenCV_INCLUDE_DIRS})

# ---------- node 3: service server, answer "where is <color>" ----------
add_executable(detect_block_server src/detect_block_server.cpp)
ament_target_dependencies(detect_block_server
  rclcpp
  std_msgs
  ifr_interfaces)

install(TARGETS image_publisher block_detector detect_block_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY images DESTINATION share/${PROJECT_NAME})

ament_package()
