#############################################################################
#
# This file is part of the ViSP software.
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

# TODO: Check if ${QT_LIBRARIES} for Qt4 should be imported using IMPORTED_LOCATION_
# TODO: Check if VTK_VERSION_BUILD or VTK_VERSION_PATH on vtk-6.x and vtk-5.x
# TODO: find how to suppress include(${VTK_USE_FILE}). For the moment, if not used on OSX, produces a segfault

#-----------------------------------------------------------------------------
# ustk version number.
set(USTK_VERSION_MAJOR "2")
set(USTK_VERSION_MINOR "0")
set(USTK_VERSION_PATCH "1")
set(USTK_VERSION "${USTK_VERSION_MAJOR}.${USTK_VERSION_MINOR}.${USTK_VERSION_PATCH}")

set(USTK_DOC_DIR "${CMAKE_BINARY_DIR}/doc-ustk")
get_filename_component(USTK_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(USTK_BINARY_DIR "${CMAKE_BINARY_DIR}")
# To include FindFFTW.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/ustk_core/cmake")

# UsTK third-party detection
VP_OPTION(USE_FFTW "FFTW" "" "Include fftw support" "" ON)
VP_OPTION(USE_ARMADILLO "Armadillo" "" "Include armadillo support" "" ON)
VP_OPTION(USE_VTK  "VTK"  QUIET "Include vtk support"  "" ON)
VP_OPTION(USE_CUDA  "CUDA"  QUIET "Include CUDA support"  "" ON)

if(USE_VTK)
  if(NOT VTK_VERSION)
    set(VTK_VERSION "${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.${VTK_BUILD_VERSION}")
  endif()
  if(${VTK_VERSION} VERSION_LESS "6.0.0")
    message("vtk-${VTK_VERSION} detected but too old to be used by ustk. You should install at least vtk-6.0.0 to use ustk_gui module.")
    set(USE_VTK OFF CACHE BOOL "Enables vtk usage" FORCE)
  endif()
endif()

if(USE_VTK)
  if("${VTK_QT_VERSION}" STREQUAL "")
    message("vtk found, but turned off since vtk was not built with Qt")
    set(USE_VTK OFF CACHE BOOL "Turn vtk support off" FORCE)
  endif()
endif()

if(NOT USE_VTK)
  #find Qt5
  #find_package(Qt5 COMPONENTS Core Network Widgets)
  VP_OPTION(USE_QT5 "Qt5;COMPONENTS;Core;Network;Widgets;PrintSupport" QUIET "Include Qt5 support" "" ON)
endif()

if(USE_VTK)
  set(USTK_HAVE_VTK_VERSION "(${VTK_VERSION_MAJOR}<<16 | ${VTK_VERSION_MINOR}<<8 | ${VTK_VERSION_PATCH})") # for usConfig.h
  if(VTK_QT_VERSION VERSION_EQUAL "5") # Qt5 not working yet
    set(USTK_HAVE_VTK_QT5 TRUE)
    set(USTK_HAVE_VTK_QT TRUE)
  elseif(VTK_QT_VERSION VERSION_EQUAL "4")
    set(USTK_HAVE_VTK_QT4 TRUE)
    set(USTK_HAVE_VTK_QT TRUE)
  endif()
endif()

if(USE_CUDA)
  set(CUDA_SEPARABLE_COMPILATION ON)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=sm_30;-lineinfo)
endif()

VP_SET(USTK_HAVE_QT5  TRUE IF USE_QT5)
VP_SET(USTK_HAVE_FFTW TRUE IF USE_FFTW)
VP_SET(USTK_HAVE_ARMADILLO TRUE IF USE_ARMADILLO)
VP_SET(USTK_HAVE_CUDA TRUE IF USE_CUDA)

#----------------------------------------------------------------------
# Use "make ustk_doc" target to generate the documentation
#----------------------------------------------------------------------
if(DOXYGEN_FOUND)
  # Note that set DOXYGEN_STRIP_FROM_INC_PATH corresponding to STRIP_FROM_INC_PATH in the
  # doxy file is already set in ViSP main CMakeLists.txt file

  configure_file(${USTK_SOURCE_DIR}/doc/config-doxygen.in
    ${USTK_DOC_DIR}/config-doxygen
    @ONLY )

  # set vars used in mainpage.doc.in
  # - VISP_MAINPAGE_EXTENSION
  set(VISP_MAINPAGE_EXTENSION "")
  set(mainpage_ext_file_ "${USTK_SOURCE_DIR}/doc/mainpage_extension.doc")
  if(EXISTS ${mainpage_ext_file_})
    file(READ ${mainpage_ext_file_} mainpage_ext_content_)
    set(VISP_MAINPAGE_EXTENSION "${VISP_MAINPAGE_EXTENSION}\n${mainpage_ext_content_}")
  endif()
  configure_file(${USTK_SOURCE_DIR}/doc/mainpage.doc.in
    ${USTK_DOC_DIR}/mainpage.doc
    @ONLY )

  add_custom_target(ustk_doc ${DOXYGEN_EXECUTABLE} ${USTK_DOC_DIR}/config-doxygen)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(ustk_doc PROPERTIES FOLDER "extra")
  endif()
endif()

# ========================== UsTK optional 3rd parties ==========================
status("")
status("  UsTK 3rd parties:")
status("    Use VTK:"           USE_VTK          THEN "yes (ver ${VTK_VERSION})" ELSE "no")
status("    Use Qt:"            USE_VTK          THEN "yes (ver ${VTK_QT_VERSION})" ELSE "no")
status("    Use FFTW:"          USE_FFTW         THEN "yes" ELSE "no")
status("    Use Armadillo:"     USE_ARMADILLO    THEN "yes (ver ${ARMADILLO_VERSION_STRING})" ELSE "no")
status("    Use CUDA:"          USE_CUDA         THEN "yes (ver ${CUDA_VERSION_STRING})" ELSE "no")


