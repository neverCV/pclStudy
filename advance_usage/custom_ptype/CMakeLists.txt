cmake_minimum_required(VERSION 3.0.0)
project(custom_ptype VERSION 0.1.0)
find_package(PCL 1.12 REQUIRED)

include_directories(PCL_INCLUDE_DIRS 
/home/lixu/Code/cppCode/pclStudy/advance_usage/custom_ptype/impl
/home/lixu/Code/cppCode/pclStudy/advance_usage/custom_ptype)
link_directories(PCL_LIBRARY_DIRS)
add_definitions(${PCL_DEFINITIONS})


# include(CTest)
enable_testing()

add_executable(custom_ptype foo.cpp)
target_link_libraries(custom_ptype ${PCL_LIBRARIES})
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

include(CPack)
