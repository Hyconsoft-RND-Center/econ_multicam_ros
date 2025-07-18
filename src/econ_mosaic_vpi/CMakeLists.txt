cmake_minimum_required(VERSION 3.8)
project(econ_mosaic_vpi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find VPI3 manually
set(VPI3_ROOT "/opt/nvidia/vpi3")
set(VPI3_INCLUDE_DIRS "${VPI3_ROOT}/include")
set(VPI3_LIBRARY_DIRS "${VPI3_ROOT}/lib/aarch64-linux-gnu")
set(VPI3_LIBRARIES "${VPI3_LIBRARY_DIRS}/libnvvpi.so")

# Check if VPI3 exists
if(NOT EXISTS ${VPI3_INCLUDE_DIRS}/vpi/VPI.h)
  message(FATAL_ERROR "VPI3 not found at ${VPI3_ROOT}. Please install VPI3.")
endif()

if(NOT EXISTS ${VPI3_LIBRARIES})
  message(FATAL_ERROR "VPI3 library not found at ${VPI3_LIBRARIES}.")
endif()

message(STATUS "Found VPI3: ${VPI3_ROOT}")
message(STATUS "VPI3 Include: ${VPI3_INCLUDE_DIRS}")
message(STATUS "VPI3 Library: ${VPI3_LIBRARIES}")

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${VPI3_INCLUDE_DIRS}
)

# Link directories
link_directories(${VPI3_LIBRARY_DIRS})

# Create mosaic view node executable
add_executable(mosaic_view_node src/mosaic_view_node.cpp)

target_link_libraries(mosaic_view_node
  ${OpenCV_LIBRARIES}
  ${VPI3_LIBRARIES}
)

ament_target_dependencies(mosaic_view_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  message_filters
)

# Install executables
install(TARGETS
  mosaic_view_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
