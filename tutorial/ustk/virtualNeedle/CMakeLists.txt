PROJECT(virtual_needle)

cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED visp_ustk_gui)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(HEADER_FILES usVirtualNeedle.h)
set(CPP_FILES usVirtualNeedle.cpp main.cpp)

include_directories(${VISP_INCLUDE_DIRS})
add_executable(virtualNeedle ${CPP_FILES} ${HEADER_FILES})
target_link_libraries(virtualNeedle ${VISP_LIBRARIES})
