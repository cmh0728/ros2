cmake_minimum_required(VERSION 3.8)
project(cpp_basic_examples)

# 컴파일러가 C++17 표준을 사용하도록 설정합니다.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(basic_publisher src/basic_publisher.cpp)
ament_target_dependencies(basic_publisher rclcpp std_msgs)

add_executable(basic_subscriber src/basic_subscriber.cpp)
ament_target_dependencies(basic_subscriber rclcpp std_msgs)

install(TARGETS
  basic_publisher
  basic_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
