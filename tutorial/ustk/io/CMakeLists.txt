project(tutorial-ustk-io)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_io)

set(tutorial_cpp
  tutorial-Rf-reader.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
  # Add specific build flag to turn off warnings coming from VTK 3rd party
  visp_set_source_file_compile_flag(${cpp} -Wno-inconsistent-missing-override)
endforeach()
