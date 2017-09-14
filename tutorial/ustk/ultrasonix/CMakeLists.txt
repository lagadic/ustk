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

project(tutorial-ultrasonix)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_grabber visp_sensor visp_robot)

set(tutorial_ultrasonix_cpp
  tutorial-ultrasonix-grabber.cpp
  tutorial-ultrasonix-confidence-2D.cpp
  tutorial-ultrasonix-servo-confidence-3D.cpp
  tutorial-ultrasonix-qt-grabbing-pre-scan.cpp
  tutorial-ultrasonix-qt-grabbing-pre-scan3D.cpp
  tutorial-ultrasonix-qt-grabbing-post-scan.cpp
  tutorial-ultrasonix-qt-grabbing-post-scan-bi-plan.cpp
  tutorial-ultrasonix-qt-grabbing-post-scan3D.cpp
  tutorial-ultrasonix-qt-grabbing-RF.cpp
  tutorial-ultrasonix-qt-grabbing-RF3D.cpp
  tutorial-ultrasonix-qt-grabbing-RF-scan-conversion.cpp
  tutorial-ultrasonix-qt-grabbing-pre-scan-confidence.cpp
  tutorial-ultrasonix-qt-grabbing-pre-scan-confidence-control.cpp)

foreach(cpp ${tutorial_ultrasonix_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

