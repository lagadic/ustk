project(tutorial-ultrasonix)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_grabber visp_sensor visp_robot)

set(tutorial_ultrasonix_cpp
  tutorial-ultrasonix-grabber.cpp
  tutorial-ultrasonix-confidence-2D.cpp
  tutorial-ultrasonix-servo-confidence-3D.cpp
  tutorial-ultrasonix-qt-grabbing-pre-scan.cpp
  tutorial-ultrasonix-qt-grabbing-post-scan.cpp
  tutorial-ultrasonix-qt-grabbing-RF.cpp
  tutorial-ultrasonix-qt-grabbing-pre-scan-confidence.cpp
  tutorial-ultrasonix-qt-grabbing-pre-scan-confidence-control.cpp)

foreach(cpp ${tutorial_ultrasonix_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

