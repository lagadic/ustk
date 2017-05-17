# TODO: Check if ${QT_LIBRARIES} for Qt4 should be imported using IMPORTED_LOCATION_
# TODO: Check if VTK_VERSION_BUILD or VTK_VERSION_PATH on vtk-6.x and vtk-5.x
# TODO: find how to suppress include(${VTK_USE_FILE}). For the moment, if not used on OSX, produces a segfault

# To include FindFFTW.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/ustk_core/cmake")

VP_OPTION(USE_FFTW "FFTW" "" "Include fftw support" "" ON)
VP_OPTION(USE_VTK "VTK" "" "Include vtk support" "" ON)

if(USE_VTK)
  if(NOT VTK_VERSION)
    set(VTK_VERSION "${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.${VTK_BUILD_VERSION}")
  endif()
  if(${VTK_VERSION} VERSION_LESS "6.0.0")
    message("vtk-${VTK_VERSION} detected but too old to be used by ustk. You should install at least vtk-6.0.0 to use ustk_gui module.")
    set(USE_VTK OFF CACHE BOOL "Enables vtk usage" FORCE)
  endif()
endif()

if(USE_VTK)
  if("${VTK_QT_VERSION}" STREQUAL "")
    message("vtk found, but turned off since vtk was not built with Qt")
    set(USE_VTK OFF CACHE BOOL "Turn vtk support off" FORCE)
  endif()
endif()

if(NOT USE_VTK)
  #find Qt5
  #find_package(Qt5 COMPONENTS Core Network Widgets)
  VP_OPTION(USE_QT5 "Qt5;COMPONENTS;Core;Network;Widgets" "" "Include Qt5 support" "" ON)
endif()

if(USE_VTK)
  set(USTK_HAVE_VTK_VERSION "(${VTK_VERSION_MAJOR}<<16 | ${VTK_VERSION_MINOR}<<8 | ${VTK_VERSION_PATCH})") # for usConfig.h
  if(VTK_QT_VERSION VERSION_EQUAL "5") # Qt5 not working yet
    set(USTK_HAVE_VTK_QT5 TRUE)
    set(USTK_HAVE_VTK_QT TRUE)
  elseif(VTK_QT_VERSION VERSION_EQUAL "4")
    set(USTK_HAVE_VTK_QT4 TRUE)
    set(USTK_HAVE_VTK_QT TRUE)
  endif()
endif()

VP_SET(USTK_HAVE_QT5         TRUE IF (BUILD_MODULE_visp_ustk_core AND USE_QT5))
VP_SET(USTK_HAVE_FFTW         TRUE IF (BUILD_MODULE_visp_ustk_core AND USE_FFTW))
