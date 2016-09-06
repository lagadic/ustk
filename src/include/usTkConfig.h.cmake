/******************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 - 2015 by Inria. All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Authors:
 * Alexandre Krupa
 * Pierre Chatelain
 *
 ******************************************************************************/

#ifndef usTkConfig_h
#define usTkConfig_h

// UsTk major version.
#define USTK_VERSION_MAJOR ${UsTk_VERSION_MAJOR}

// UsTk minor version.
#define USTK_VERSION_MINOR ${UsTk_VERSION_MINOR}

// UsTk patch version.
#define USTK_VERSION_PATCH ${UsTk_VERSION_PATCH}

// UsTk version.
#define USTK_VERSION_FULL ${UsTk_VERSION_FULL}

// Defined if MSVC is the compiler
#cmakedefine USTK_USE_MSVC

// Ustk library is either compiled static or shared
// Used to set declspec(import, export) in headers if required under Windows
#cmakedefine USTK_BUILD_SHARED_LIBS

// Defined if ViSP library available.
#cmakedefine USTK_HAVE_VISP

// Defined if ViSP library available.
#cmakedefine USTK_USE_OPTIMIZED_SNAKE

// Defined if CUDA available.
#cmakedefine USTK_HAVE_CUDA

// Defined if CUDA version >= 5 available (the cutil library does not exist anymore and helper_cuda.h should be used instead)
#cmakedefine USTK_HAVE_HELPER_CUDA

// Defined if CULA available.
#cmakedefine USTK_HAVE_CULA

// Defined if Levmar available.
#cmakedefine USTK_HAVE_LEVMAR

// Defined if Ultrasonix available
#cmakedefine USTK_HAVE_ULTRASONIX

// Defined if V4L2 available
#cmakedefine USTK_HAVE_V4L2

// Defined if the IIWA robot controller is available
#cmakedefine USTK_HAVE_IIWA

// Defined if the Robots module is available
#cmakedefine USTK_HAVE_ROBOTS

// Path to the configuration files
#define USTK_CONFIG_PATH "${UsTk_CONFIG_PATH}"

// Path to Porta's probes.xml file
#ifdef USTK_HAVE_ULTRASONIX
#define US_PORTA_CONFIG_PROBES_FILENAME "${US_PORTA_CONFIG_PROBES_FILENAME}"
#endif

// Under Windows, for shared libraries (DLL) we need to define export on
// compilation or import on use (like a third party project).
// We exploit here the fact that cmake auto set xxx_EXPORTS (with S) on
// compilation.
#if defined (WIN32) && defined(USTK_BUILD_SHARED_LIBS)
#  ifdef UsTk_EXPORTS
#    define USTK_EXPORT __declspec(dllexport)
#  else
#    define USTK_EXPORT __declspec(dllimport)
#  endif
#else
#  define USTK_EXPORT
#endif

#endif
