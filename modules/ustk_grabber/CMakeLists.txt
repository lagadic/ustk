VP_OPTION(USE_VTK "VTK;COMPONENTS;vtkGUISupportQt;" "" "Include vtk support" "" ON)

  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)
    find_package(Qt5 COMPONENTS Widgets Network REQUIRED QUIET)
    qt5_wrap_ui(UI_Srcs ${UIs})

    list(APPEND opt_incs "${Qt5Core_INCLUDE_DIRS}")
    list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")
    list(APPEND opt_incs "${Qt5Network_INCLUDE_DIRS}")

    list(APPEND opt_libs Qt5::Widgets Qt5::Core Qt5::Network)


  vp_add_module(ustk_grabber visp_gui visp_ustk_io )
  vp_glob_module_sources()
  vp_module_include_directories(${opt_incs})
  vp_create_module(${opt_libs})

  vp_add_tests()


    set(USTK_HAVE_QT5 TRUE)
vp_add_config_file("cmake/templates/usGrabberConfig.h.in")
