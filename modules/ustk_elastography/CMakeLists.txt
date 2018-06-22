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
# Marc Pouliquen
# Pedro Patlan Rosales
#
#############################################################################

if(USE_FFTW)
  list(APPEND opt_incs ${FFTW_INCLUDE_DIRS})
  list(APPEND opt_libs ${FFTW_LIBRARIES})

  if(USE_ARMADILLO)
    list(APPEND opt_incs ${ARMADILLO_INCLUDE_DIRS})
    list(APPEND opt_libs ${ARMADILLO_LIBRARIES})
  endif()

  vp_add_module(ustk_elastography visp_ustk_core)

  vp_glob_module_sources()

  vp_module_include_directories(${opt_incs})
  vp_create_module(${opt_libs})

  vp_add_tests(DEPENDS_ON visp_ustk_core)

  if(APPLE)
    vp_set_source_file_compile_flag(src/usElastography.cpp -Wno-unused-parameter)
    vp_set_source_file_compile_flag(src/usMotionEstimation.cpp -Wno-unused-parameter)
    vp_set_source_file_compile_flag(test/testUsElastography.cpp -Wno-unused-parameter)
  endif()

endif()
