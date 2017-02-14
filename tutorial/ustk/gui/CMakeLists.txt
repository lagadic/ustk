project(tutorial-medical-image-viewer)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_gui visp_ustk_io)

set(tutorial_cpp
  usMedicalViewerExample.cpp
  usFullConverter.cpp
  usGrabber3DViewer.cpp
  us3DViewer.cpp
  us2DViewer.cpp
  usResliceMatrixExample.cpp
  us3DViewerSlicing.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Add specific build flag to turn off warnings coming from VTK 3rd party
visp_set_source_file_compile_flag(usFullConverter.cpp -Wno-inconsistent-missing-override)
visp_set_source_file_compile_flag(usMedicalViewerExample.cpp -Wno-inconsistent-missing-override)
