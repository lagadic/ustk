#############################################################################
#
# $Id: FindLEVMAR.cmake,v 1.4 2009/02/17 16:13:06 akrupa Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# Try to find libraries for Levmar algorithm 
# Once run this will define: 
#
# LEVMAR_FOUND
# LEVMAR_INCLUDE_DIR
# LEVMAR_LIBRARIES
#
# Authors:
# Alexandre Krupa
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindLEVMAR.cmake: LEVMAR only available for Unix.")
  SET(LEVMAR_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(LEVMAR_INCLUDE_DIR lm.h
    $ENV{LEVMAR_HOME}
    $ENV{LEVMAR_HOME}/include
        )
  #MESSAGE("DBG LEVMAR_INCLUDE_DIR=${LEVMAR_INCLUDE_DIR}")  
    
  FIND_LIBRARY(LEVMAR_LIBRARY
    NAMES levmar
    PATHS 
    $ENV{LEVMAR_HOME}/lib
    $ENV{LEVMAR_HOME}/build
        )

  #MESSAGE("DBG LEVMAR_LIBRARY=${LEVMAR_LIBRARY}")
 
   
  ## --------------------------------
  
  IF(LEVMAR_LIBRARY)
    SET(LEVMAR_LIBRARIES ${LEVMAR_LIBRARY})
  ELSE(LEVMAR_LIBRARY)
    MESSAGE(SEND_ERROR "Levmar library not found.")
  ENDIF(LEVMAR_LIBRARY)
  
  IF(LEVMAR_INCLUDE_DIR)
    SET(LEVMAR_INCLUDE_DIR ${LEVMAR_INCLUDE_DIR})
  ELSE(LEVMAR_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "Levmar include dir not found.")
  ENDIF(LEVMAR_INCLUDE_DIR)
  
  IF(LEVMAR_LIBRARIES AND LEVMAR_INCLUDE_DIR)
    SET(LEVMAR_FOUND TRUE)
    #----------------------------------------------------------------------
    # Add definitions
    #----------------------------------------------------------------------
    ADD_DEFINITIONS("-DHAVE_LEVMAR")
  ELSE(LEVMAR_LIBRARIES AND LEVMAR_INCLUDE_DIR)
    SET(LEVMAR_FOUND FALSE)
  ENDIF(LEVMAR_LIBRARIES AND LEVMAR_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    LEVMAR_INCLUDE_DIR
    LEVMAR_LIBRARIES
    LEVMAR_LIBRARY
    )
ENDIF(NOT UNIX)
