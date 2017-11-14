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
# Description:
# Try to find porta SDK (tested with porta 5.6.0).
# Once run this will define:
#
# Porta_FOUND
# Porta_INCLUDE_DIRS
# Porta_LIBRARIES
# Porta_SETTINGS_PATH
# Porta_FIRMWARE_PATH
#
# Authors:
# Fabien Spindler
# Marc Pouliquen
#
#############################################################################

find_path(Porta_INCLUDE_DIRS porta.h
  $ENV{PORTA_DIR}/inc
)

find_library(Porta_LIBRARIES
  NAMES porta
  PATHS 
  $ENV{PORTA_DIR}/lib
)

find_path(Porta_FIRMWARE_PATH
  NAMES dl-2-1.bit
  PATHS 
  $ENV{PORTA_DIR}/fw
)

find_path(Porta_SETTINGS_PATH
  NAMES "config/filters.txt"
  PATHS 
  $ENV{PORTA_DIR}/dat
)

if(Porta_LIBRARIES AND Porta_INCLUDE_DIRS)
  set(Porta_FOUND TRUE)
  message(STATUS "Porta found")
else()
  set(Porta_FOUND FALSE)
  message(STATUS "Could not find Porta (if you are on a ulstrasonix machine, make sure you properly filled PORTA_DIR environment variable).")
endif()
 
mark_as_advanced(
  Porta_INCLUDE_DIRS
  Porta_LIBRARIES
  Porta_FIRMWARE_PATH
  Porta_SETTINGS_PATH
)
