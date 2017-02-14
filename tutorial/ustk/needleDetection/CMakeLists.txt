project(tutorial-needle-detection-2D)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_core visp_ustk_io visp_ustk_needle_detection)

set(tutorial_cpp
  needleDetection2DExample.cpp
  volReading.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

