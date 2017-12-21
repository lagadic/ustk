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
# Pedro Patlan Rosales
#
#############################################################################

if(USE_FFTW)
  list(APPEND opt_incs ${FFTW_INCLUDE_DIRS})
  list(APPEND opt_libs ${FFTW_LIBRARIES})
endif()

if(USE_ARMADILLO)
  list(APPEND opt_incs ${ARMADILLO_INCLUDE_DIRS})
  list(APPEND opt_libs ${ARMADILLO_LIBRARIES})
endif()

if(USE_QT5)
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)

  list(APPEND opt_incs "${Qt5Core_INCLUDE_DIRS}")
  list(APPEND opt_incs "${Qt5Network_INCLUDE_DIRS}")
  list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")
  list(APPEND opt_incs "${Qt5PrintSupport_INCLUDE_DIRS}")

  if(${Qt5Network_FOUND})
    list(APPEND opt_libs Qt5::Core Qt5::Network)
  endif()

  if(${Qt5Widgets_FOUND})
    list(APPEND opt_libs Qt5::Widgets)
  endif()

  if(${Qt5PrintSupport_FOUND})
    list(APPEND opt_libs Qt5::PrintSupport)
  endif()
endif()

if(USE_VTK)
  include(${VTK_USE_FILE})

  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)

  # Find Qt version corresponding to the Qt version used to compile VTK (4 or 5)
  if(VTK_QT_VERSION VERSION_EQUAL "5") # Qt5 not working yet
    find_package(Qt5 COMPONENTS Widgets Network PrintSupport OpenGL REQUIRED QUIET)
    qt5_wrap_ui(UI_Srcs ${UIs})

    list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")
    list(APPEND opt_libs Qt5::Widgets Qt5::Network)

    list(APPEND opt_incs "${Qt5PrintSupport_INCLUDE_DIRS}")
    list(APPEND opt_libs Qt5::PrintSupport)

    list(APPEND opt_incs "${Qt5OpenGL_INCLUDE_DIRS}")
    list(APPEND opt_libs Qt5::OpenGL)

    foreach(lib_ ${Qt5Network_LIBRARIES})
      foreach(imp_config_ ${config_})
        get_target_property(lib_property_${imp_config_}_ ${lib_} IMPORTED_LOCATION_${imp_config_})
        # Under Unix, there is no specific suffix for PCL/VTK libraries.         # Under Windows, we add the "optimized", "debug" specific keywords
        if(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
          list(APPEND opt_libs optimized "${component_property_${imp_config_}_}")
        elseif(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
          list(APPEND opt_libs debug     "${lib_property_${imp_config_}_}")
        elseif(EXISTS "${lib_property_${imp_config_}_}")
          list(APPEND opt_libs "${lib_property_${imp_config_}_}")
        endif()
        get_target_property(lib_deps_${imp_config_}_ ${lib_} IMPORTED_LINK_INTERFACE_LIBRARIES_${imp_config_})

        if(lib_deps_${imp_config_}_)
          foreach(deps_ ${lib_deps_${imp_config_}_})
            get_target_property(deps_location_ ${deps_} IMPORTED_LOCATION_${imp_config_})
            if(EXISTS "${deps_location_}")
              if(WIN32 AND "${imp_config_}" MATCHES "RELEASE")
                list(APPEND opt_libs optimized ${deps_location_})
              elseif(WIN32 AND "${imp_config_}" MATCHES "DEBUG")
                list(APPEND opt_libs debug ${deps_location_})
              else()
                list(APPEND opt_libs ${deps_location_})
              endif()
            endif()
          endforeach()
        endif()

      endforeach()
    endforeach()

  elseif(VTK_QT_VERSION VERSION_EQUAL "4")
    find_package(Qt4 REQUIRED QtGui QtCore)
    # Use what VTK built with
    set(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")
    set(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
    set(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")

    list(APPEND opt_incs "${QT_QTGUI_INCLUDE_DIR}")
    #list(APPEND opt_incs "${QT_INCLUDE_DIR}/QtCore") # seems not useful
    list(APPEND opt_libs ${QT_QTGUI_LIBRARIES})
    list(APPEND opt_libs ${QT_QTCORE_LIBRARIES})
    list(APPEND opt_libs ${QT_QTNETWORK_LIBRARIES})

  endif()
endif()

vp_add_module(ustk_elastography visp_ustk_core)

vp_glob_module_sources()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
