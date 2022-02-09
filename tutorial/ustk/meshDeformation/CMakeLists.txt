project(mesh_deformation)

cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED visp_ustk_gui)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(HEADER_FILES usMeshDeformation.h)
set(CPP_FILES usMeshDeformation.cpp tutorial-mesh-deformation.cpp)

include_directories(${VISP_INCLUDE_DIRS})
add_executable(meshDeformation ${CPP_FILES} ${HEADER_FILES})
target_link_libraries(meshDeformation ${VISP_LIBRARIES})
