project(tutorial-medical-image-viewer)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_gui visp_ustk_io)

set(tutorial_cpp
  usMedicalViewerExample.cpp
  usFullConverter.cpp
  usVTKconverter.cpp
  usGrabber3DViewer.cpp
  us3DViewer.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
