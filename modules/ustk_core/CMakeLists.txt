
vp_add_module(ustk_core visp_core)

vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()

vp_add_tests()

vp_add_config_file("cmake/templates/usConfig.h.in")
