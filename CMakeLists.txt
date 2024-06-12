cmake_minimum_required(VERSION 3.15)
project(multi_sensor_collector)

# 选项
option(BUILD_UTILS "Build utils." OFF)

# 编译和输出配置
set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
elseif (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
elseif (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")
endif ()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 可执行文件
include_directories(${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/core/*.cpp
    ${PROJECT_SOURCE_DIR}/src/plugin/*.cpp
    ${PROJECT_SOURCE_DIR}/src/fileio/*.cpp
)
add_library(multi_sensor_collector SHARED
    ${SRC_FILES}
)

# Glog
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
target_link_libraries(multi_sensor_collector ${GLOG_LIBRARIES})

# abseil-cpp
set(ABSL_PROPAGATE_CXX_STD ON)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/abseil-cpp)
target_link_libraries(multi_sensor_collector
    absl::strings
    absl::str_format
    absl::time
)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(multi_sensor_collector ${YAML_CPP_LIBRARIES})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# HighFive
set(HIGHFIVE_USE_EIGEN ON)
set(HIGHFIVE_UNIT_TESTS OFF)
set(HIGHFIVE_EXAMPLES OFF)
set(HIGHFIVE_BUILD_DOCS OFF)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/HighFive)
target_link_libraries(multi_sensor_collector HighFive)

# TBB
find_package(TBB REQUIRED)
target_link_libraries(multi_sensor_collector TBB::tbb)

# serial
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/serial)
target_link_libraries(multi_sensor_collector serial)

# OpenCV
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)
find_package(OpenCV REQUIRED)
target_link_libraries(multi_sensor_collector ${OpenCV_LIBS})

# MetavisionSDK
find_package(MetavisionSDK COMPONENTS driver core REQUIRED)
target_link_libraries(multi_sensor_collector MetavisionSDK::driver MetavisionSDK::core)

# 可执行文件
add_executable(multi_sensor_collector_exe
    ${PROJECT_SOURCE_DIR}/src/main.cpp
)
target_link_libraries(multi_sensor_collector_exe multi_sensor_collector)

# 构建工具
if (BUILD_UTILS)
    add_subdirectory(${PROJECT_SOURCE_DIR}/utils)
endif ()