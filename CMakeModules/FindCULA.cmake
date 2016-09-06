#############################################################################
#
# $Id: FindCULA.cmake,v 1.3 2008-12-19 14:24:13 fspindle Exp $
#
# Copyright (C) 2010 Inria. All rights reserved.
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
# Try to find CULA library.
#
# CULA_FOUND
# CULA_INCLUDE_DIR
# CULA_LIBRARY
# CULA_LAPACK_LIBRARY
# CULA_LAPACK_LINK_LIBRARY
# CULA_LIBRARIES
#
#############################################################################

  
  FIND_PATH(CULA_INCLUDE_DIR cula.h
  $ENV{CULA_ROOT}/include
    )
  
  FIND_LIBRARY(CULA_LIBRARY
	NAMES cula cula_core
   	PATHS 
    $ENV{CULA_ROOT}/lib 
    )

 FIND_LIBRARY(CULA_LAPACK_LIBRARY
	NAMES culapack cula_lapack
   	PATHS 
    $ENV{CULA_ROOT}/lib 
    )

 FIND_LIBRARY(CULA_LAPACK_LINK_LIBRARY
	NAMES cula_lapack_link
   	PATHS 
    $ENV{CULA_ROOT}/lib 
    )
  
  
  ## --------------------------------
  
  IF(CULA_LIBRARY)
    SET(CULA_LIBRARIES ${CULA_LIBRARY})
  ELSE(CULA_LIBRARY)
#    MESSAGE(SEND_ERROR "CULA library not found.")
  ENDIF(CULA_LIBRARY)

 IF(CULA_LAPACK_LIBRARY)
    SET(CULA_LIBRARIES ${CULA_LAPACK_LIBRARY} ${CULA_LIBRARIES})
  ELSE(CULA_LAPACK_LIBRARY)
#    MESSAGE(SEND_ERROR "CULA_LAPACK library not found.")
  ENDIF(CULA_LAPACK_LIBRARY)

 IF(CULA_LAPACK_LINK_LIBRARY)
    SET(CULA_LIBRARIES ${CULA_LAPACK_LINK_LIBRARY} ${CULA_LIBRARIES})
  ELSE(CULA_LAPACK_LINK_LIBRARY)
#    MESSAGE(SEND_ERROR "CULA_LAPACK_LINK library not found.")
  ENDIF(CULA_LAPACK_LINK_LIBRARY)
  
  IF(NOT CULA_INCLUDE_DIR)
#    MESSAGE(SEND_ERROR "CULA include dir not found.")
  ENDIF(NOT CULA_INCLUDE_DIR)
  
  IF(CULA_LIBRARIES AND CULA_INCLUDE_DIR)
    SET(CULA_FOUND TRUE)
  ELSE(CULA_LIBRARIES AND CULA_INCLUDE_DIR)
    SET(CULA_FOUND FALSE)
  ENDIF(CULA_LIBRARIES AND CULA_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    CULA_INCLUDE_DIR
    CULA_LIBRARIES
    CULA_LIBRARY
    CULA_LAPACK_LIBRARY
    )
