vp_add_module(ustk_gui INTERNAL visp_core visp_io visp_ustk_core)

vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()

vp_add_tests(CTEST_EXCLUDE_PATH DEPENDS_ON visp_core visp_io visp_ustk_core visp_ustk_io)
