if(UNIX)
  #ultrasonix grabber not available for window
  vp_add_module(ustk_grabber visp_core visp_io visp_ustk_core visp_ustk_io)

  vp_glob_module_sources()
  vp_module_include_directories()
  vp_create_module()

  vp_add_tests()
endif()

