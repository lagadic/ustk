###############################################################################
#
# This file is part of the UsTk software.
# Copyright (C) 2014 by Inria. All rights reserved.
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

#----------------------------------------------------------------------
# Include project files 
#----------------------------------------------------------------------
# include all the Example project .cpp files
INCLUDE(${UsTk_SOURCE_DIR}/CMakeSourceFileList.cmake)
# include all the Example project .h files
INCLUDE(${UsTk_SOURCE_DIR}/CMakeHeaderFileList.cmake)


#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/UsTk
#----------------------------------------------------------------------
# For each header, we create a rule
SET(HEADER_IN_INCLUDE_DIR "")
SET(ADDED_SRC_DEPEND_PROPERTIES "")
FOREACH(header_ ${HEADER_ALL})
  GET_FILENAME_COMPONENT(headerName ${header_} NAME) 
  ADD_CUSTOM_COMMAND(
    OUTPUT ${UsTk_BINARY_DIR}/include/UsTk/${headerName}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header_} ${UsTk_BINARY_DIR}/include/UsTk/${headerName}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header_}
    )
  SET(HEADER_IN_INCLUDE_DIR ${HEADER_IN_INCLUDE_DIR} ${UsTk_BINARY_DIR}/include/UsTk/${headerName}
    )
ENDFOREACH(header_)

#----------------------------------------------------------------------
# Create a top level rule to copy all the headers from src to include/UsTk
#----------------------------------------------------------------------
ADD_CUSTOM_TARGET(header ALL
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )


#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------
IF(CUDA_FOUND)
	CUDA_ADD_LIBRARY(${UsTk_INTERN_LIBRARY}
		${SRC_ALL}
		${HEADER_ALL}
		${UsTk_INCLUDE_DIR}/usTkConfig.h) 
ELSE(CUDA_FOUND)
	ADD_LIBRARY (${UsTk_INTERN_LIBRARY}
		${SRC_ALL}
		${HEADER_ALL}
		${UsTk_INCLUDE_DIR}/usTkConfig.h) 
ENDIF(CUDA_FOUND)

# create the headers in include/UsTk before compiling the lib
ADD_DEPENDENCIES(${UsTk_INTERN_LIBRARY} header)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${UsTk_INTERN_LIBRARY} PROPERTIES
  VERSION ${UsTk_VERSION}
  SOVERSION ${UsTk_VERSION_MAJOR}.${UsTk_VERSION_MINOR}
  PUBLIC_HEADER "${HEADER_ALL};${UsTk_INCLUDE_DIR}/usTkConfig.h"
  OUTPUT_NAME "${UsTk_INTERN_LIBRARY}"
  DEBUG_POSTFIX "${UsTk_DEBUG_POSTFIX}"
  RUNTIME_OUTPUT_DIRECTORY "${BINARY_OUTPUT_PATH}"
  LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
  ARCHIVE_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
  )
  
SET_PROPERTY(TARGET ${UsTk_INTERN_LIBRARY} APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${UsTk_EXTERN_INCLUDE_DIRS}
)

# Link external libraries
TARGET_LINK_LIBRARIES(${UsTk_INTERN_LIBRARY} ${UsTk_EXTERN_LIBS})

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL(TARGETS ${UsTk_INTERN_LIBRARY}
  EXPORT UsTkTargets
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/UsTk
  RUNTIME DESTINATION ${UsTk_BIN_INSTALL_PATH} COMPONENT libraries
  LIBRARY DESTINATION ${UsTk_LIB_INSTALL_PATH} COMPONENT libraries
  ARCHIVE DESTINATION ${UsTk_LIB_INSTALL_PATH} COMPONENT libraries
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
)

# install rule for all the headers
# INSTALL_FILES(/include/UsTk FILES ${HEADER_ALL})
