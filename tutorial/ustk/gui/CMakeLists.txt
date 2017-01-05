project(tutorial-scan-conversion)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_core visp_ustk_io)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
set(tutorial_cpp
  QtVTKRenderWindowsApp.cxx)

foreach(cpp ${tutorial_cpp})
  #visp_add_target(${cpp})
  include_directories(${VISP_INCLUDE_DIRS})
  include_directories(${QT_INCLUDE_DIR})
  add_executable(ustkGuiTutorial ${cpp})
  target_link_libraries(ustkGuiTutorial ${VISP_LIBRARIES} ${QT_LIBRARIES})
endforeach()

