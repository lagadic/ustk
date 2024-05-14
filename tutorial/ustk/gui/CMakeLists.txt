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

cmake_minimum_required(VERSION 3.5)

project(tutorial-medical-image-viewer)

find_package(VISP REQUIRED visp_ustk_gui)

set(tutorial_cpp
  usMedicalViewerExample.cpp
  usFullConverter.cpp
  usGrabber3DViewer.cpp
  us3DViewer.cpp
  us2DViewer.cpp
  us2DViewerDrawing.cpp
  us2DViewerPicker.cpp
  usResliceMatrixExample.cpp
  us3DViewerSlicing.cpp
  tutorial-image-display-qwidget.cpp
  tutorial-image-display-qtquickoverlay.cpp
  tutorial-image-display-qtquickoverlayServoing.cpp
  tutorial-robot-control-gui.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
  # Add specific build flag to turn off warnings coming from VTK 3rd party
  visp_set_source_file_compile_flag(${cpp} -Wno-inconsistent-missing-override)
endforeach()
