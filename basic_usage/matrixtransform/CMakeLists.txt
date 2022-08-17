cmake_minimum_required(VERSION 3.0.0)
project(matrix_transform VERSION 0.1.0)
find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})#头文件包含
link_directories(${PCL_LIBRARY_DIRS})#链接目录包含

add_definitions(${PCL_DEFINITIONS})# 添加宏定义

enable_testing()

add_executable(matrix_transform matrix_transform.cpp)

target_link_libraries(matrix_transform ${PCL_LIBRARIES})# 需要链接的动态库

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
