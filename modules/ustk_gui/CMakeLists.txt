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

if(USE_VTK)
  include(${VTK_USE_FILE})

  list(APPEND opt_incs ${VTK_INCLUDE_DIRS})
  vp_get_interface_link_libraries(VTK_LIBRARIES vtk_link_libraries_)
#  message("vtk_link_libraries_: ${vtk_link_libraries_}")
  list(APPEND opt_libs ${vtk_link_libraries_})

  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)

  # Find Qt version corresponding to the Qt version used to compile VTK (4 or 5)
  if(VTK_QT_VERSION VERSION_EQUAL "5")
    find_package(Qt5 COMPONENTS Gui Widgets QuickWidgets REQUIRED QUIET)
    qt5_wrap_ui(UI_Srcs ${UIs})

    qt5_add_resources(UI_RESOURCES resources/resources.qrc)
    set_property(SOURCE ${UI_RESOURCES} PROPERTY SKIP_AUTOMOC ON) # To avoid CMP0071 warning

    list(APPEND opt_incs "${Qt5Gui_INCLUDE_DIRS}")
    list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")
    list(APPEND opt_incs "${Qt5QuickWidgets_INCLUDE_DIRS}")

    # The following line allows to build visp, but doesn't allow to build a project that uses
    # visp as 3rd party:
    # list(APPEND opt_libs ${Qt5Widgets_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5QuickWidgets_LIBRARIES})
    # Instead we get the corresponding interface link libraries that we can propagate with visp
    # when visp is used as a 3rd party:
    set(Qt5_COMPONENTS ${Qt5Widgets_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5QuickWidgets_LIBRARIES})
    vp_get_interface_link_libraries(Qt5_COMPONENTS qt5_link_libraries_)
#    message("qt5_link_libraries_: ${qt5_link_libraries_}")
    list(APPEND opt_libs ${qt5_link_libraries_})

  elseif(VTK_QT_VERSION VERSION_EQUAL "4")
    find_package(Qt4 REQUIRED QtGui QtCore OPTIONAL_COMPONENTS QtNetwork)
    # Use what VTK built with
    set(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")
    set(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
    set(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")

    list(APPEND opt_incs "${QT_INCLUDE_DIR}")
    list(APPEND opt_incs "${QT_QTGUI_INCLUDE_DIR}")
    list(APPEND opt_incs "${QT_QTCORE_INCLUDE_DIR}")
    list(APPEND opt_incs "${QT_QTNETWORK_INCLUDE_DIR}")
    list(APPEND opt_libs ${QT_QTGUI_LIBRARIES})
    list(APPEND opt_libs ${QT_QTCORE_LIBRARIES})
    list(APPEND opt_libs ${QT_QTNETWORK_LIBRARIES})
  endif()

  # Manage resources file (for qml components)
  set(AUTORCC_ON)

  # Create ustk_gui module, only if vtk found
  vp_add_module(ustk_gui visp_ustk_core visp_ustk_grabber visp_robot visp_ustk_confidence_map visp_ustk_template_tracking OPTIONAL visp_ustk_elastography)
  vp_glob_module_sources(${UI_RESOURCES})
  vp_module_include_directories(${opt_incs})
  vp_create_module(${opt_libs})

  vp_add_tests()

  # Add specific build flag to turn off warnings coming from VTK 3rd party
  vp_set_source_file_compile_flag(src/us2DSceneWidget.cpp -Wno-inconsistent-missing-override)
  vp_set_source_file_compile_flag(src/us3DSceneSlicing.cpp -Wno-inconsistent-missing-override)
  vp_set_source_file_compile_flag(src/us3DSceneWidget.cpp -Wno-inconsistent-missing-override)
  vp_set_source_file_compile_flag(src/usMedicalImageViewer.cpp -Wno-inconsistent-missing-override)
  vp_set_source_file_compile_flag(src/usResliceMatrixViewer.cpp -Wno-inconsistent-missing-override)
  vp_set_source_file_compile_flag(src/usViewerWidget.cpp -Wno-inconsistent-missing-override)
  vp_set_source_file_compile_flag(src/usVTKConverter.cpp -Wno-inconsistent-missing-override)

  vp_set_source_file_compile_flag(test/testUsVTKconverter.cpp -Wno-inconsistent-missing-override)

  if(APPLE)
    vp_set_source_file_compile_flag(src/usElastographyQtWrapper.cpp -Wno-unused-parameter)
    vp_set_source_file_compile_flag(src/usImageElastographyCreationWrapper.cpp -Wno-unused-parameter)
  endif()

endif()
