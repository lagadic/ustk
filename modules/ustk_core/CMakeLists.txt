#############################################################################
#
# This file is part of the ustk software.
# Copyright (C) 2016 - 2017 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ustk with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at ustk@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Authors:
# Fabien Spindler
#
#############################################################################

if(USE_FFTW)
  list(APPEND opt_incs ${FFTW_INCLUDE_DIRS})
  list(APPEND opt_libs ${FFTW_LIBRARIES})
endif()

IF(CUDA_FOUND)
  IF(NOT VISP_INITIAL_PASS)
    CUDA_INCLUDE_DIRECTORIES(${VISP_MODULE_visp_ustk_core_LOCATION}/include)
    CUDA_INCLUDE_DIRECTORIES(${VISP_MODULE_visp_ustk_core_INC_DEP})
    foreach(d ${VISP_MODULE_visp_ustk_core_DEPS})
      if(d MATCHES "^visp_" AND HAVE_${d})
        if(EXISTS "${VISP_MODULE_${d}_LOCATION}/include")
          CUDA_INCLUDE_DIRECTORIES(${VISP_MODULE_${d}_LOCATION}/include)
        endif()
          CUDA_INCLUDE_DIRECTORIES(${VISP_MODULE_${d}_INC_DEP})
      endif()
    endforeach()
    CUDA_ADD_LIBRARY(ustk_core_cuda  ./src/scanConversion/usPreScanToPostScan3DConverter.cu STATIC)
    vp_install_target(ustk_core_cuda EXPORT VISPModules OPTIONAL
    RUNTIME DESTINATION ${VISP_BIN_INSTALL_PATH} COMPONENT libs
    LIBRARY DESTINATION ${VISP_LIB_INSTALL_PATH} COMPONENT libs
    ARCHIVE DESTINATION ${VISP_LIB_INSTALL_PATH} COMPONENT dev
    )
  ENDIF(NOT VISP_INITIAL_PASS)
  list(APPEND opt_libs ustk_core_cuda)
ENDIF(CUDA_FOUND)

vp_add_module(ustk_core visp_io)

vp_glob_module_sources()

vp_module_include_directories(${opt_incs})

vp_create_module(${opt_libs})

vp_add_tests(DEPENDS_ON visp_ustk_core visp_gui)

vp_add_config_file("cmake/templates/usConfig.h.in")

# export USTK_HAVE_* cmake vars in VISPConfig.cmake
vp_cmake_configure("cmake/templates/VISPConfig-ustk_core.cmake.in")

vp_set_source_file_compile_flag(src/geometry/usBSpline3D.cpp -Wno-unused-but-set-variable)