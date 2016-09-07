vp_add_module(ustk_grabber visp_core)

vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()

vp_add_tests(DEPENDS_ON visp_core visp_io visp_ustk_data)
