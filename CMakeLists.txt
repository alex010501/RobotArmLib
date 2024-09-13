# CMake version
cmake_minimum_required(VERSION 3.15)
# Project name
project(RobotArmLib)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build folders
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})

# External libraries
# Eigen
set(EIGEN_DIR ${RobotArmLib_SOURCE_DIR}/extern/Eigen)
add_subdirectory(extern/Eigen)

# Set 'RAL_SOURCES' variable
file(GLOB_RECURSE RAL_SOURCES CONFIGURE_DEPENDS "include/*.h"
                                                "src/*.cpp")

# Build RAL library 
add_library(RobotArmLib STATIC ${RAL_SOURCES})

# Set include directories
target_include_directories(RobotArmLib PUBLIC ${RobotArmLib_SOURCE_DIR}/include)

# Link libraries
target_link_libraries(RobotArmLib Eigen3::Eigen)