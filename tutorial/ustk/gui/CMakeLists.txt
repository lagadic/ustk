project(tutorial-image-viewer)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_gui)

set(tutorial_cpp
  usMedicalViewerExample.cpp)

foreach(cpp ${tutorial_cpp})
  #visp_add_target(${cpp})
  include_directories(${VISP_INCLUDE_DIRS})
  add_executable(usMedicalViewerExample ${cpp})
  target_link_libraries(usMedicalViewerExample ${VISP_LIBRARIES})
endforeach()
