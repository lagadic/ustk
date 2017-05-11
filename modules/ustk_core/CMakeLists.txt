
# CMake Modules Configuration
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(FFTW)
if(FFTW_FOUND)
  message(STATUS "FOUND FFTW")
  list(APPEND opt_libs ${FFTW_LIBRARIES})
  list(APPEND opt_incs ${FFTW_INCLUDES})
endif()

VP_SET(USTK_HAVE_FFTW         TRUE IF (BUILD_MODULE_visp_ustk_core AND FFTW_FOUND))

vp_add_module(ustk_core visp_core)

vp_glob_module_sources()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

vp_add_tests()

vp_add_config_file("cmake/templates/usConfig.h.in")
