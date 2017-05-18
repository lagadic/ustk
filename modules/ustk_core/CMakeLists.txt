if(USE_FFTW)
  list(APPEND opt_incs ${FFTW_INCLUDE_DIRS})
  list(APPEND opt_libs ${FFTW_LIBRARIES})
endif()

vp_add_module(ustk_core visp_core)

vp_glob_module_sources()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

vp_add_tests()

vp_add_config_file("cmake/templates/usConfig.h.in")
