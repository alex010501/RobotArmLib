# CMake version
cmake_minimum_required(VERSION 3.15)
# Project name
project(RobotArmLib)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build folders
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})

# Check subproject or not
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Building RobotArmLib as main project")
    # Set vcpkg toolchain
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake CACHE STRING "Vcpkg toolchain file")
endif()

# External libraries
# Eigen
find_package(Eigen3 CONFIG REQUIRED)
# set(EIGEN_DIR ${RobotArmLib_SOURCE_DIR}/extern/Eigen)
# add_subdirectory(extern/Eigen)

# Set 'RAL_SOURCES' variable
file(GLOB_RECURSE RAL_SOURCES CONFIGURE_DEPENDS "include/*.h"
                                                "src/*.cpp")

# Build RAL library 
add_library(RobotArmLib STATIC ${RAL_SOURCES})

# Set include directories
target_include_directories(RobotArmLib PUBLIC ${RobotArmLib_SOURCE_DIR}/include)

# Link libraries
target_link_libraries(RobotArmLib PRIVATE Eigen3::Eigen)