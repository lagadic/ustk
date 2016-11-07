project(tutorial-sonosite)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_core visp_ustk_confidence_map)

set(tutorial_cpp
  tutorial-sonosite-confidence-map.cpp
  tutorial-sonosite-grabber.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

