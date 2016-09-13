vp_add_module(ustk_core INTERNAL visp_core visp_io)


set(USTK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()

vp_add_tests(DEPENDS_ON visp_core visp_io)