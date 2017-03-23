cmake_minimum_required(VERSION 2.8.2)

project(augment3d)

# Download and unpack gflags at configure time
configure_file(CMakeLists.gflags.txt gflags-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/gflags-download )
if(result)
  message(FATAL_ERROR "CMake step for gflags failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/gflags-download )
if(result)
  message(FATAL_ERROR "Build step for gflags failed: ${result}")
endif()

# Add gflags directly to our build. This defines
# the gflags_static and gflags_nothreads_static targets.
add_subdirectory(${CMAKE_BINARY_DIR}/gflags-src
                 ${CMAKE_BINARY_DIR}/gflags-build)


# Download and unpack googletest at configure time
configure_file(CMakeLists.gtest.txt googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()


# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Boost
find_package(Boost 1.5 COMPONENTS regex filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})


# Main binary
file(GLOB AUGMENT3D_SRC augment3d/*.h augment3d/*.cpp)
list(REMOVE_ITEM AUGMENT3D_SRC ${CMAKE_CURRENT_SOURCE_DIR}/augment3d/augment3d.cpp)
add_executable(augment3d augment3d/augment3d.cpp ${AUGMENT3D_SRC})
target_link_libraries(augment3d ${PCL_LIBRARIES})
target_link_libraries(augment3d ${Boost_LIBRARIES})
target_link_libraries(augment3d gflags_static)


# Tests
file(GLOB AUGMENT3D_TEST_SRC augment3d/test/*.cpp)
add_executable(unittests ${AUGMENT3D_TEST_SRC} ${AUGMENT3D_SRC})
target_link_libraries(unittests ${PCL_LIBRARIES})
target_link_libraries(unittests ${Boost_LIBRARIES})
target_link_libraries(unittests gtest_main)
