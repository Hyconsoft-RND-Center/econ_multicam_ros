cmake_minimum_required(VERSION 3.8)
project(econ_ros LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Compiler settings
set(VERSION_STRING "1.0-g57a8186")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -pedantic -fcommon -DVERSION=\"1.0-g57a8186\"")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -pedantic -std=c++14")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lpthread -Wl,--allow-multiple-definition")



# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(X11 REQUIRED x11)

# VPI library removed - using GStreamer hardware acceleration instead

# Find NvBufSurface libraries (JetPack 6.2.0)
find_library(NVBUFSURFACE_LIB nvbufsurface PATHS /usr/lib/aarch64-linux-gnu/nvidia REQUIRED)
if(NOT NVBUFSURFACE_LIB)
    message(FATAL_ERROR "nvbufsurface library not found")
endif()

find_library(NVBUFSURFTRANSFORM_LIB nvbufsurftransform PATHS /usr/lib/aarch64-linux-gnu/nvidia REQUIRED)
if(NOT NVBUFSURFTRANSFORM_LIB)
    message(FATAL_ERROR "nvbufsurftransform library not found")
endif()

# find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Include directories and link libraries
include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${X11_INCLUDE_DIRS}
    /usr/src/jetson_multimedia_api/include
    src
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/econ_ros
)

# Link libraries globally like the original
link_libraries(
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${X11_LIBRARIES}
    ${NVBUFSURFACE_LIB}
    ${NVBUFSURFTRANSFORM_LIB}
    m
    jpeg
    pthread
)

# Source files
file(GLOB C_FILES "src/*.c")
file(GLOB CPP_FILES "src/*.cpp")
add_executable(econ_ros ${C_FILES} ${CPP_FILES})

# Include the GStreamer ROS publisher
target_sources(econ_ros PRIVATE
    src/gst_ros_publisher.c
)

# Dependency settings
set_source_files_properties(${C_FILES} PROPERTIES LANGUAGE C)
set_source_files_properties(${CPP_FILES} PROPERTIES LANGUAGE CXX)
target_include_directories(econ_ros PRIVATE /usr/include/gstreamer-1.0 src)

# ROS2 dependencies for the executable
ament_target_dependencies(econ_ros
  rcl
  rclc
  sensor_msgs
  std_msgs
  builtin_interfaces
)





# Install the executable
install(TARGETS econ_ros
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Custom commands
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/*.o econ_ros
)

ament_package()
