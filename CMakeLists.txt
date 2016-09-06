###############################################################################
#
# This file is part of the UsTk software.
# Copyright (C) 2014 - 2015 by Inria. All rights reserved.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License ("GPL") as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
# See the file COPYING at the root directory of this source
# distribution for additional information about the GNU GPL.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact the
# authors at Alexandre.Krupa@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
#
# Authors:
# Alexandre Krupa
# Pierre Chatelain
#
###############################################################################

PROJECT(UsTk)

# ccmake version that works
cmake_minimum_required(VERSION 2.6)

#-----------------------------------------------------------------------------
# UsTk version number.  An even minor number corresponds to releases.
SET(UsTk_VERSION_MAJOR 1)
SET(UsTk_VERSION_MINOR 0)
SET(UsTk_VERSION_PATCH 1)
SET(UsTk_VERSION "${UsTk_VERSION_MAJOR}.${UsTk_VERSION_MINOR}")
SET(UsTk_VERSION_FULL "${UsTk_VERSION}.${UsTk_VERSION_PATCH}")      

# the include directory we depend on
SET(UsTk_INTERN_INCLUDE_DIR ${UsTk_BINARY_DIR}/include)
SET(UsTk_INCLUDE_DIR "${UsTk_BINARY_DIR}/include/UsTk")
SET(UsTk_DOC_DIR "${UsTk_BINARY_DIR}/doc")

#--------------------------------------------------------------------
# By default set release configuration
#--------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
endif()


#-----------------------------------------------------------------------------
Macro(remove_cxx_flag flag)
  string(REPLACE "${flag}" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endmacro()

SET(CMAKE_CXX_STANDARD 11)

# The location in which to install Example libraries.
SET(LIBRARY_OUTPUT_PATH ${UsTk_BINARY_DIR}/lib)

# The location in which to install UsTk binaries.
SET(BINARY_OUTPUT_PATH ${UsTk_BINARY_DIR}/bin)

# the third party include directories we depend on
SET(UsTk_EXTERN_INCLUDE_DIR "")
# the internal visp libs we want to createn
SET(UsTk_INTERN_LIBRARY UsTk)
# the third party libraries we depend on
SET(UsTk_EXTERN_LIBS "")
# the third party libraries we depend on
SET(UsTk_DEFS "")

if(WIN32)
  # Postfix of .lib and .dll
  set(UsTk_DEBUG_POSTFIX "d")
  set(UsTk_DLLVERSION "${VISP_VERSION_MAJOR}${VISP_VERSION_MINOR}${VISP_VERSION_PATCH}")
else()
  set(UsTk_DEBUG_POSTFIX "")
  set(UsTk_DLLVERSION "")
endif()

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${UsTk_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${UsTk_INCLUDE_DIR})

#--------------------------------------------------------------------
# Option management
#--------------------------------------------------------------------

# Choose static or shared libraries.
OPTION(BUILD_SHARED_LIBS "Build UsTk with shared libraries." ON)

# Optimized or standard version of the snake.
#OPTION(BUILD_OPTIMIZED_SNAKE "Use optimized snake." OFF)

# Option for building RF processing tools
#OPTION(MODULE_RF "Build RF processing module" OFF)

# Option for building tests
#OPTION(BUILD_TESTING "Build tests" OFF)

# Option for building examples
#"OPTION(BUILD_EXAMPLES "Build examples" OFF)

# Use VTL
#OPTION(USE_VTK "Use the VTK library." OFF)

# Use CUDA SDK
#OPTION(USE_CUDA_SDK "Use CUDA SDK." OFF)

# Use CULA SDK
#OPTION(USE_CULA_SDK "Use CULA SDK." OFF)

# Use LEVMAR
#OPTION(USE_LEVMAR "Use LEVMAR." OFF)

# Grabbers
#OPTION(USE_ULTRASONIX "Build Ultrasonix interface." OFF)
#OPTION(USE_V4L2 "Build V4L2 grabber." OFF)

# Use Robots
#OPTION(MODULE_ROBOTS "Build the Robots module" OFF)

#IF(MODULE_ROBOTS)
#  OPTION(USE_VIPER650 "Compile UsTk for Viper S650 robot usage at Irisa" OFF)
#  OPTION(USE_VIPER850 "Compile UsTk for Viper S850 robot usage at Irisa" OFF)
#  OPTION(USE_IIWA "Compile UsTK for LBR iiwa 7 robot usage at IFL" OFF)
#ENDIF()

if(MSVC)
  option(BUILD_WITH_STATIC_CRT "Enables use of statically linked CRT for statically linked UsTk" ON)
endif()

IF(BUILD_SHARED_LIBS)
  SET(USTK_BUILD_SHARED_LIBS TRUE)  # for header usTkConfig.h
ENDIF(BUILD_SHARED_LIBS)

#IF(BUILD_OPTIMIZED_SNAKE)
#  SET(USTK_USE_OPTIMIZED_SNAKE TRUE)  # for header usTkConfig.h
#ENDIF(BUILD_OPTIMIZED_SNAKE)

#SET(USTK_HAVE_ROBOTS FALSE)

# Path to the config files
SET(UsTk_CONFIG_PATH ${UsTk_SOURCE_DIR}/config CACHE PATH "Path to the configuration files")

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(UsTk_DEFS "${UsTk_DEFS} -DWIN32")
  SET(UsTk_DEFS "${UsTk_DEFS} -DGX") # To support exceptions
  SET(UsTk_DEFS "${UsTk_DEFS} -DNOMINMAX") # Disable windows MIN/MAX
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(UsTk_EXTERN_LIBS ${UsTk_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(UsTk_DEFS "${UsTk_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(UsTk_DEFS "${UsTk_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------

# CUDA
# IF(USE_CUDA_SDK)
  # IF(NOT APPLE)
    # FIND_PACKAGE(MyCUDA QUIET REQUIRED)
  # ELSE()
    # FIND_PACKAGE(CUDA QUIET REQUIRED)
  # ENDIF()
  # IF(CUDA_FOUND)
    # MESSAGE(STATUS "Found CUDA")
    # IF(CUDA_VERSION_MAJOR LESS 5)
      # #IF(CUDA_CUT_INCLUDE_DIR)
      # #  MESSAGE("CUDA_CUTIL found.")
      # #  MESSAGE("You have an old cuda version < 5.0 " )
      # #  MESSAGE("Please updrage your cuda toolkit/library" )
      # SET(UsTk_EXTERN_INCLUDE_DIR ${CUDA_CUT_INCLUDE_DIR} ${UsTk_EXTERN_INCLUDE_DIR})
      # SET(UsTk_EXTERN_LIBS ${UsTk_EXTERN_LIBS} ${CUDA_LIBRARIES} ${CUDA_CUT_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} )
    # ELSE()
      # #ELSEIF(EXISTS ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc/helper_cuda.h)
      # # MESSAGE("CUDA_CUTIL not found but helper_cuda.h found.")
      # # Message("You have a cuda version >= 5.0")
      # # MESSAGE("Cutil does not exist anymore in this version and you should replace/remove calls to cutil functions from your sources" )
      # # For cuda version >= 5.0
      # SET(UsTk_EXTERN_INCLUDE_DIR ${CUDA_SDK_ROOT_DIR}/common/inc ${UsTk_EXTERN_INCLUDE_DIR})
      # SET(UsTk_EXTERN_LIBS ${UsTk_EXTERN_LIBS} ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} )
      # SET(UsTk_EXTERN_INCLUDE_DIR ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc ${UsTk_EXTERN_INCLUDE_DIR})
      # SET(USTK_HAVE_HELPER_CUDA TRUE)
    # ENDIF()
    # SET(USTK_HAVE_CUDA TRUE)
    # if (APPLE)
      # # Add nvcc flag for compatibility with CUDA 5.5 and later
      # set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-ccbin=/usr/bin/clang")
    # endif()
  # ENDIF(CUDA_FOUND)
# ENDIF(USE_CUDA_SDK)

# CULA
# IF(USE_CULA_SDK)
# FIND_PACKAGE(CULA QUIET REQUIRED)
# IF(CULA_FOUND)
  # MESSAGE(STATUS "Found CULA")
  # SET(UsTk_EXTERN_INCLUDE_DIR ${CULA_INCLUDE_DIR} ${UsTk_EXTERN_INCLUDE_DIR})
  # SET(UsTk_EXTERN_LIBS ${UsTk_EXTERN_LIBS} ${CULA_LIBRARIES})
  # SET(USTK_HAVE_CULA TRUE)
# ENDIF(CULA_FOUND)
# ENDIF(USE_CULA_SDK)

# Levmar libraries
# IF(USE_LEVMAR)
  # FIND_PACKAGE(LEVMAR)
  # IF(LEVMAR_FOUND)
    # SET(UsTk_EXTERN_INCLUDE_DIR ${LEVMAR_INCLUDE_DIR} ${UsTk_EXTERN_INCLUDE_DIR})
    # SET(UsTk_EXTERN_LIBS ${LEVMAR_LIBRARIES} ${UsTk_EXTERN_LIBS})
    # SET(USTK_HAVE_LEVMAR TRUE)
  # ELSE(LEVMAR_FOUND)
    # MESSAGE("Levmar requested but not found.")
   # ENDIF(LEVMAR_FOUND)
# ENDIF(USE_LEVMAR)  

# ViSP
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  MESSAGE(STATUS "Found ViSP")
  INCLUDE(${VISP_USE_FILE})
  SET(USTK_HAVE_VISP TRUE)
  IF(MODULE_ROBOTS)
    IF(VISP_HAVE_VIPER650 OR VISP_HAVE_VIPER850)
      SET(USTK_HAVE_ROBOTS TRUE)
    ENDIF()
  ENDIF()
ENDIF(VISP_FOUND)

# VTK libraries
# IF (USE_VTK)
  # FIND_PACKAGE(VTK REQUIRED)
  # IF(VTK_FOUND)
    # MESSAGE(STATUS "Found VTK")
    # INCLUDE(${VTK_USE_FILE})
    # SET(USTK_HAVE_VTK TRUE)
    # SET(UsTk_EXTERN_LIBS ${VTK_LIBRARIES} ${UsTk_EXTERN_LIBS})
  # ENDIF(VTK_FOUND)
# ENDIF()

# FFTW
# IF(MODULE_RF)
  # FIND_PACKAGE(FFTW REQUIRED)
  # IF(FFTW_FOUND)
    # MESSAGE(STATUS "Found FFTW")
    # SET(UsTk_EXTERN_LIBS ${FFTW_LIBRARIES} ${UsTk_EXTERN_LIBS})
  # ENDIF(FFTW_FOUND)
# ENDIF(MODULE_RF)

# Check Ultrasonix dependencies
# IF(USE_ULTRASONIX)
	# IF(WIN32 AND NOT CYGWIN)
		# MESSAGE(SEND_ERROR "Ultrasonix grabber is not implemented for Windows.")
	# ELSE()
	  # SET(USTK_HAVE_ULTRASONIX TRUE)
	  # FIND_PATH(US_PORTA_CONFIG_PROBES_FILENAME probes.xml)
	# ENDIF()
# ENDIF()

# Check whether ViSP has V4L2
IF(USE_V4L2)
  IF(NOT VISP_HAVE_V4L2)
    MESSAGE(SEND_ERROR "ViSP was built without V4L2.")
  ELSE()
    SET(USTK_HAVE_V4L2 TRUE)
  ENDIF()
ENDIF()

# UsRobotIIWA
# IF(USE_IIWA)
  # FIND_PACKAGE(UsRobotIIWA REQUIRED)
  # IF(UsRobotIIWA_FOUND)
    # MESSAGE(STATUS "Found UsRobotIIWA")
    # FOREACH(lib ${UsRobotIIWA_LIBRARIES})
      # GET_TARGET_PROPERTY(UsRobotIIWA_LIB ${lib} "LOCATION")
      # SET(UsTk_EXTERN_LIBS ${UsTk_EXTERN_LIBS} ${UsRobotIIWA_LIB})
    # ENDFOREACH()
    # SET(UsTk_EXTERN_INCLUDE_DIR ${UsTk_EXTERN_INCLUDE_DIR} ${UsRobotIIWA_INCLUDE_DIRS})
    # SET(USTK_HAVE_IIWA TRUE)
    # IF(MODULE_ROBOTS)
      # SET(USTK_HAVE_ROBOTS TRUE)
    # ENDIF()
  # ENDIF()
# ENDIF()

# Check whether at least one robot was found
# IF(MODULE_ROBOTS AND (NOT USTK_HAVE_ROBOTS))
  # MESSAGE(SEND_ERROR "Module Robots requires at least one robot, but no robot was found.")
# ENDIF()

# OpenMP
# FIND_PACKAGE(OpenMP)
# IF(OPENMP_FOUND)
  # LIST(APPEND CMAKE_C_FLAGS ${OpenMP_C_FLAGS})
  # LIST(APPEND CMAKE_CXX_FLAGS ${OpenMP_CXX_FLAGS})
# ENDIF()

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
# With Visual Studio 2005, Microsoft deprecates the standard C library, for
# example fopen() and sprintf(), to non-portable functions fopen_s() and
# sprintf_s(). These functions are considered by Microsoft more secure. This is
# a worthwhile exercise ! The use of these deprecated functions causes a lot of
# warnings. To suppress it, we add the _CRT_SECURE_NO_DEPRECATE preprocessor
# definition
IF(WIN32 AND MSVC)
  ADD_DEFINITIONS("-D_CRT_SECURE_NO_DEPRECATE")
ENDIF()

IF(UsTk_DEFS)
  LIST(REMOVE_DUPLICATES UsTk_DEFS)
ENDIF(UsTk_DEFS)
ADD_DEFINITIONS(${UsTk_DEFS})

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
IF(UsTk_EXTERN_INCLUDE_DIR)
	LIST(REMOVE_DUPLICATES UsTk_EXTERN_INCLUDE_DIR)
ENDIF()
INCLUDE_DIRECTORIES(BEFORE ${UsTk_INTERN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${UsTk_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
#LINK_DIRECTORIES(${UsTk_BINARY_DIR}/lib)

# where to install the library
if(WIN32)
  if(UsTk_STATIC)
    set(UsTk_LIB_INSTALL_PATH "lib")
  else()
    set(UsTk_LIB_INSTALL_PATH "lib")
  endif()
  set(UsTk_BIN_INSTALL_PATH "bin")
else()
  set(UsTk_LIB_INSTALL_PATH "lib")
  set(UsTk_BIN_INSTALL_PATH "bin")
endif()

#----------------------------------------------------------------------
# add html-doc target for doxygen documentation building
#----------------------------------------------------------------------
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)  
  ## we need latex for doxygen because of the formulas
  FIND_PACKAGE(LATEX)
  IF    (NOT LATEX_COMPILER)
    MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user inetraction on doxy run.")
  ENDIF (NOT LATEX_COMPILER)
  IF    (NOT MAKEINDEX_COMPILER)
    MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  ENDIF (NOT MAKEINDEX_COMPILER)
  IF    (NOT DVIPS_CONVERTER)
    MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  ENDIF (NOT DVIPS_CONVERTER)
   
  CONFIGURE_FILE(${UsTk_SOURCE_DIR}/doc/config-doxygen.cmake
    ${UsTk_SOURCE_DIR}/doc/config-doxygen
    @ONLY )
  ADD_CUSTOM_TARGET(html-doc ${DOXYGEN_EXECUTABLE} ${UsTk_SOURCE_DIR}/doc/config-doxygen) 
ENDIF(DOXYGEN_FOUND)

#----------------------------------------------------------------------
# Enable testing
#----------------------------------------------------------------------
ENABLE_TESTING()

#----------------------------------------------------------------------
# Propagation in sub dirs
#----------------------------------------------------------------------

# Library
ADD_SUBDIRECTORY(src)

#----------------------------------------------------------------------
# Generate the package dependent file include/UsTk/usTkConfig.h
#----------------------------------------------------------------------
CONFIGURE_FILE(${UsTk_SOURCE_DIR}/include/usTkConfig.h.cmake 
  ${UsTk_INCLUDE_DIR}/usTkConfig.h
)

# Testing
IF(BUILD_TESTING)
  SET(UsTk_DATA_PATH ${UsTk_SOURCE_DIR}/data CACHE PATH "Path to the test data")
  ADD_SUBDIRECTORY(tests)
ENDIF(BUILD_TESTING)

# Examples
IF(BUILD_EXAMPLES)
  ADD_SUBDIRECTORY(examples)
ENDIF(BUILD_EXAMPLES)

# Apps
# ADD_SUBDIRECTORY(tools)

#----------------------------------------------------------------------
# Export the library
#----------------------------------------------------------------------
EXPORT(TARGETS ${UsTk_INTERN_LIBRARY} FILE "${UsTk_BINARY_DIR}/UsTkTargets.cmake")
 
EXPORT(PACKAGE UsTk)
 
SET(CONF_INCLUDE_DIRS ${UsTk_INTERN_INCLUDE_DIR} ${UsTk_EXTERN_INCLUDE_DIR})
CONFIGURE_FILE(UsTkConfig.cmake.in
  "${UsTk_BINARY_DIR}/UsTkConfig.cmake" @ONLY)

CONFIGURE_FILE(UsTkConfigVersion.cmake.in
  "${UsTk_BINARY_DIR}/UsTkConfigVersion.cmake" @ONLY)

# Remove c++11 flag for compatibility with CUDA 5.5 and later
# IF(USTK_HAVE_CUDA)
  # remove_cxx_flag("-std=c++11")
# ENDIF(USTK_HAVE_CUDA)
