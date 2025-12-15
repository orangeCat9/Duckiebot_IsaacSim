cmake_minimum_required(VERSION 3.8)
project(duckie_led)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. 서비스 파일 생성 설정
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetColor.srv"
)

# 2. 파이썬 스크립트 설치 설정
install(PROGRAMS
  scripts/led_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
