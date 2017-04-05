VP_OPTION(USE_VTK "VTK;COMPONENTS;vtkGUISupportQt;" "" "Include vtk support" "" ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#find Qt5
find_package(Qt5 COMPONENTS Core Network)

list(APPEND opt_incs "${Qt5Core_INCLUDE_DIRS}")
list(APPEND opt_incs "${Qt5Network_INCLUDE_DIRS}")

list(APPEND opt_libs Qt5::Core Qt5::Network)

vp_add_module(ustk_grabber visp_gui visp_ustk_io )
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

vp_add_tests()

if(${Qt5Network_FOUND})
  set(USTK_HAVE_QT5 TRUE)
endif()

vp_add_config_file("cmake/templates/usGrabberConfig.h.in")
