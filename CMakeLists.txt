cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(augment3d)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (augment3d augment3d.cpp transforms.cpp)
target_link_libraries (augment3d ${PCL_LIBRARIES})
