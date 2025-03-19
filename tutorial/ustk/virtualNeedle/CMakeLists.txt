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

cmake_minimum_required(VERSION 3.10)

project(virtual_needle)

find_package(VISP REQUIRED visp_ustk_gui)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(HEADER_FILES usVirtualNeedle.h)
set(CPP_FILES usVirtualNeedle.cpp main.cpp)

include_directories(${VISP_INCLUDE_DIRS})
add_executable(virtualNeedle ${CPP_FILES} ${HEADER_FILES})
target_link_libraries(virtualNeedle ${VISP_LIBRARIES})
