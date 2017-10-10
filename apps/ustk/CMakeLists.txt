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

cmake_minimum_required(VERSION 2.6)

set(CMAKE_AUTOMOC ON)

# If virtual server is built with VISP
if(${CMAKE_PROJECT_NAME} STREQUAL "VISP")
  VP_OPTION(BUILD_USTK_APPS      "" "" "Build ustk apps such as virtual server" "" OFF)
  set(VISP_DIR ${PROJECT_BINARY_DIR})
  set(CMAKE_CURRENT_BINARY_DIR ${CMAKE_BINARY_DIR}/apps/ustk)
  message(STATUS "CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}")
endif()

if(BUILD_USTK_APPS)
  find_package(VISP REQUIRED visp_ustk_io visp_ustk_grabber)

  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Network REQUIRED)

  include_directories(${VISP_INCLUDE_DIRS})

  set(HEADER_FILES usVirtualServer.h)
  set(CPP_FILES usVirtualServer.cpp virtualServer.cpp)

  add_executable(ustk-virtualServer ${CPP_FILES} ${HEADER_FILES})

  target_link_libraries(ustk-virtualServer Qt5::Widgets Qt5::Gui visp_ustk_io visp_ustk_grabber)
endif()
