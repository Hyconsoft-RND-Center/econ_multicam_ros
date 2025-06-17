cmake_minimum_required(VERSION 3.10)
project(e-multicam VERSION 1.0)

set(PARA_ROOT $ENV{PARA_ROOT})

# Compiler settings
set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc")
set(CMAKE_CXX_COMPILER "${CROSS_COMPILE}g++")
set(VERSION_STRING "1.0-g57a8186")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -pedantic -DVERSION=\"1.0-g57a8186\"")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -pedantic -std=c++14")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lpthread")

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(X11 REQUIRED x11)

find_library(PARA_CORE_LIB para_core PATHS ${PARA_ROOT}/lib)
find_library(PARA_EXEC_LIB para_exec PATHS ${PARA_ROOT}/lib)
find_library(PARA_COM_LIB para_com PATHS ${PARA_ROOT}/lib)
find_library(PARA_LOG_LIB para_log PATHS ${PARA_ROOT}/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories and link libraries
include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${X11_INCLUDE_DIRS}
    ${PARA_ROOT}/include
)

link_libraries(
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${X11_LIBRARIES}
    m
    jpeg
)

# Source files
file(GLOB C_FILES "*.c")
file(GLOB CPP_FILES "*.cpp")
add_executable(e-multicam ${C_FILES} ${CPP_FILES})

# Dependency settings
set_source_files_properties(${C_FILES} PROPERTIES LANGUAGE C)
set_source_files_properties(${CPP_FILES} PROPERTIES LANGUAGE CXX)
target_include_directories(e-multicam PRIVATE /usr/include/gstreamer-1.0 ${PARA_ROOT}/include)

# Set compile definitions
#target_compile_definitions(e-multicam PRIVATE VERSION="\\\"${VERSION_STRING}\\\"")

# Link pthread library explicitly

# Find and link libraries
target_link_libraries(e-multicam pthread ${PARA_CORE_LIB} ${PARA_EXEC_LIB} ${PARA_COM_LIB} ${PARA_LOG_LIB})

# Link the executable
ament_target_dependencies(e-multicam
  rclc
  sensor_msgs
)

# Custom commands
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/*.o e-multicam
)

ament_package()
