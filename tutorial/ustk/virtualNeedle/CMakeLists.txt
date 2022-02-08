PROJECT(virtual_needle)
 
cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED visp_ustk_gui)

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Gui)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(VISP REQUIRED visp_ustk_gui)

include_directories(${VISP_INCLUDE_DIRS})

set(HEADER_FILES usVirtualNeedle.h)
set(CPP_FILES usVirtualNeedle.cpp main.cpp)

add_executable(vitrualNeedle ${CPP_FILES} ${HEADER_FILES})

target_link_libraries(vitrualNeedle Qt5::Widgets Qt5::Gui visp_ustk_gui ${VTK_LIBRARIES})
