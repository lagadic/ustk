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

cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_AUTOMOC ON)

find_package(VISP REQUIRED visp_ustk_core)

if(USTK_HAVE_VTK_QT5 OR USTK_HAVE_QT5)
  find_package(Qt5Widgets)
  find_package(Qt5Network)

  if(Qt5Widgets_FOUND AND Qt5Network_FOUND)

    include_directories(${VISP_INCLUDE_DIRS})

    set(HEADER_FILES usVirtualServer.h usConsoleListener.h)
    set(CPP_FILES usVirtualServer.cpp virtualServer.cpp usConsoleListener.cpp)

    add_executable(ustk-virtualServer ${CPP_FILES} ${HEADER_FILES})

    target_link_libraries(ustk-virtualServer Qt5::Widgets Qt5::Network ${VISP_LIBRARIES})

  endif()
endif()

