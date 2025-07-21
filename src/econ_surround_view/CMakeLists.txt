cmake_minimum_required(VERSION 3.8)
project(econ_surround_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 표준 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcl_interfaces REQUIRED)

# OpenCV 찾기
find_package(OpenCV 4 REQUIRED)

# VPI 찾기 (수동으로 설정)
set(VPI_INCLUDE_DIR "/opt/nvidia/vpi3/include")
set(VPI_LIBRARY "/opt/nvidia/vpi3/lib/aarch64-linux-gnu/libnvvpi.so")

# 포함 디렉토리 설정
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${VPI_INCLUDE_DIR}
)

# 실행 파일 생성
add_executable(surround_view_node
  src/surround_view_node.cpp
  src/vpi_image_processor.cpp
  src/camera_calibration_manager.cpp
  src/surround_view_stitcher.cpp
)

# 의존성 설정
ament_target_dependencies(surround_view_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
  rcl_interfaces
)

# 링크 라이브러리 설정
target_link_libraries(surround_view_node
  ${OpenCV_LIBS}
  ${VPI_LIBRARY}
)

# 설치
install(TARGETS
  surround_view_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Config 파일 설치
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
