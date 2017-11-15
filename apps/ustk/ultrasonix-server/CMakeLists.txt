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
#
#############################################################################

project(ustk-ultrasonix-server)

cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#if(POLICY CMP0020)
  #cmake_policy(SET CMP0020 NEW)
#endif()

find_package(Porta)

if(Porta_FOUND)
  OPTION(USE_SONIXTOUCH "Set to ON if you want to use the Sonixtouch" ON)
  if(USE_SONIXTOUCH)
    add_definitions(-DSONIXTOUCH)
  endif()
  set(Porta_LICENCE_PATH "" CACHE PATH "Path to the licenses_web.txt file containing your licence")

  include_directories(${Porta_INCLUDE_DIRS})
  add_definitions(-DSETTINGS_PATH=\"${Porta_SETTINGS_PATH}/\")
  add_definitions(-DFIRMWARE_PATH=\"${Porta_FIRMWARE_PATH}/\")
  
  find_package(VISP REQUIRED visp_core)
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Network REQUIRED)

  include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
  include_directories(${VISP_INCLUDE_DIRS})

  file(GLOB UI_FILES *.ui)
  file(GLOB QT_WRAP *.h)
  file(GLOB CPP_FILES *.cpp)

  QT5_WRAP_UI(UISrcs ${UI_FILES})
  QT5_WRAP_CPP(MOCSrcs ${QT_WRAP})
  add_executable(ustk-ultrasonixServer MACOSX_BUNDLE ${CPP_FILES} ${UISrcs} ${MOCSrcs})

  target_link_libraries(ustk-ultrasonixServer Qt5::Widgets Qt5::Network ${Porta_LIBRARIES} )

  configure_file("cmake/templates/usPortaConfig.h.in" "usPortaConfig.h")
endif()
