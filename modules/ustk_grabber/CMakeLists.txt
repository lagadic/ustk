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

if(USE_QT5)
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)

  list(APPEND opt_incs "${Qt5Core_INCLUDE_DIRS}")
  list(APPEND opt_incs "${Qt5Network_INCLUDE_DIRS}")
  list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")

  # The following line allows to build visp, but doesn't allow to build a project that uses
  # visp as 3rd party:
  # list(APPEND opt_libs ${Qt5Core_LIBRARIES} ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES})
  # Instead we get the corresponding interface link libraries that we can propagate with visp
  #  when visp is used as a 3rd party:
  set(Qt5_COMPONENTS ${Qt5Core_LIBRARIES} ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES})
  vp_get_interface_link_libraries(Qt5_COMPONENTS qt5_link_libraries_)
#  message("qt5_link_libraries_: ${qt5_link_libraries_}")
  list(APPEND opt_libs ${qt5_link_libraries_})
endif()

if(USE_VTK)
  #include(${VTK_USE_FILE})

  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)

  # Find Qt version corresponding to the Qt version used to compile VTK (4 or 5)
  if(VTK_QT_VERSION VERSION_EQUAL "5") # Qt5 not working yet
    find_package(Qt5 COMPONENTS Widgets Network REQUIRED QUIET)
    qt5_wrap_ui(UI_Srcs ${UIs})

    list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")
    list(APPEND opt_incs "${Qt5Network_INCLUDE_DIRS}")

    # The following line allows to build visp, but doesn't allow to build a project that uses
    # visp as 3rd party:
    # list(APPEND opt_libs ${Qt5Widgets_LIBRARIES} ${Qt5Network_LIBRARIES})
    # Instead we get the corresponding interface link libraries that we can propagate with visp
    #  when visp is used as a 3rd party:
    set(Qt5_COMPONENTS ${Qt5Widgets_LIBRARIES} ${Qt5Network_LIBRARIES})
    vp_get_interface_link_libraries(Qt5_COMPONENTS qt5_link_libraries_)
#    message("qt5_link_libraries_: ${qt5_link_libraries_}")
    list(APPEND opt_libs ${qt5_link_libraries_})

  elseif(VTK_QT_VERSION VERSION_EQUAL "4")
    find_package(Qt4 REQUIRED QtGui QtCore)
    # Use what VTK built with
    set(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")
    set(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
    set(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")

    list(APPEND opt_incs "${QT_QTGUI_INCLUDE_DIR}")
    list(APPEND opt_incs "${QT_QTCORE_INCLUDE_DIR}")
    list(APPEND opt_incs "${QT_QTGUI_INCLUDE_DIR}")
    list(APPEND opt_libs ${QT_QTGUI_LIBRARIES})
    list(APPEND opt_libs ${QT_QTCORE_LIBRARIES})
    list(APPEND opt_libs ${QT_QTNETWORK_LIBRARIES})

  endif()
endif()

if(USE_QT5 OR USE_VTK)
  vp_add_module(ustk_grabber visp_ustk_core)
  vp_glob_module_sources()
  vp_module_include_directories(${opt_incs})
  vp_create_module(${opt_libs})

  vp_add_tests()
endif()
