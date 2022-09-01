cmake_minimum_required(VERSION 3.0.0)
project(momentOfInertia VERSION 0.1.0)
include(CTest)

find_package(PCL 1.12 REQUIRED)
message(${PCL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

enable_testing()

add_executable(momentOfInertia momentOfInertia.cpp)
target_link_libraries(momentOfInertia ${PCL_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
