cmake_minimum_required(VERSION 3.10)

# Project name and version
project(FixedWingGNC VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Release)

# Include directories
add_subdirectory(include/plog)
include_directories(${CMAKE_SOURCE_DIR}/include)

#Eigen has to be installed
#Download here, then extract the zip file
#https://gitlab.com/libeigen/eigen/-/releases/5.0.0
find_package (Eigen3 REQUIRED NO_MODULE)

#Include source files
set(SOURCES
    src/Aircraft.cpp
    src/NavGuidance.cpp
    src/Simulator.cpp
)

add_executable(FixedWingGNC main.cpp ${SOURCES})

#Link plog library
target_link_libraries(FixedWingGNC plog::plog Eigen3::Eigen)