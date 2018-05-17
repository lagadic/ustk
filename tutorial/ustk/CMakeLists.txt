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

project(visp-tutorial-ustk)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

if(MSVC)
  if(NOT VISP_SHARED)
    foreach(flag_var
            CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
            CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif()
      if(${flag_var} MATCHES "/MDd")
        string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
      endif()
    endforeach(flag_var)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:msvcrtd.lib")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:libcmt.lib")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /NODEFAULTLIB:libcmtd.lib")
  endif()
endif()

visp_add_subdirectory(confidence REQUIRED_DEPS visp_ustk_core visp_ustk_confidence_map)
visp_add_subdirectory(io REQUIRED_DEPS visp_ustk_core)
visp_add_subdirectory(gui REQUIRED_DEPS visp_ustk_gui visp_ustk_core)
visp_add_subdirectory(needleDetection REQUIRED_DEPS visp_ustk_core visp_ustk_needle_detection)
visp_add_subdirectory(imageConversion REQUIRED_DEPS visp_ustk_core)
visp_add_subdirectory(sonosite REQUIRED_DEPS visp_ustk_confidence_map visp_ustk_template_tracking visp_robot)
visp_add_subdirectory(tracking REQUIRED_DEPS visp_ustk_confidence_map visp_ustk_template_tracking visp_ustk_grabber visp_robot)
visp_add_subdirectory(ultrasonix REQUIRED_DEPS visp_ustk_grabber)
visp_add_subdirectory(virtualServerClients REQUIRED_DEPS visp_ustk_grabber)
visp_add_subdirectory(realTimeDisplay REQUIRED_DEPS visp_ustk_gui visp_ustk_grabber)
visp_add_subdirectory(elastography REQUIRED_DEPS visp_ustk_elastography)
if(USTK_HAVE_VTK_QT5)
  visp_add_subdirectory(virtualNeedle REQUIRED_DEPS visp_ustk_gui)
endif()
