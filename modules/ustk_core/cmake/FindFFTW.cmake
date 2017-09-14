#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2016 - 2017 by Inria. All rights reserved.
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

# Find the native fftw includes and libraries
#
# FFTW_FOUND - True if fftw found.
# FFTW_INCLUDE_DIRS - where to find fftw3.h
# FFTW_LIBRARIES - List of libraries when using FFTW.

if(FFTW_INCLUDE_DIRS)
  # Already in cache, be silent
  set(FFTW_FIND_QUIETLY TRUE)
endif(FFTW_INCLUDE_DIRS)

if(WIN32)
  find_path(FFTW_INCLUDE_DIRS fftw3.h
    $ENV{FFTW_HOME}
  )
  find_library(FFTW_LIBRARIES NAMES libfftw3-3
    $ENV{FFTW_HOME}
  )
else()
  if($ENV{FFTW_HOME})
    find_path(FFTW_INCLUDE_DIRS fftw3.h
      "$ENV{FFTW_HOME}/include"
    )

    find_library(FFTW_LIBRARIES NAMES "fftw3"
      PATHS "$ENV{FFTW_HOME}/lib"
    )
  else()
    find_path(FFTW_INCLUDE_DIRS fftw3.h)
    find_library(FFTW_LIBRARIES NAMES fftw3)
  endif()
endif()

# handle the QUIETLY and REQUIRED arguments and set FFTW_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFTW DEFAULT_MSG FFTW_LIBRARIES FFTW_INCLUDE_DIRS)
mark_as_advanced(FFTW_LIBRARIES FFTW_INCLUDE_DIRS)
