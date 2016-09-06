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

# Set SRC_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update SRC_ALL variable if you add/remove a SRC_subdir 
# variable
#
# If you add/remove a directory, modify here

# SET (SRC_BI_FD_SNAKE
  # snake/usBiFDSnake/usBiSnake.cpp
  # )

# SET (SRC_CONTROLLER
  # controller/usRobotController.cpp
  # controller/usWorkingPoint.cpp
  # controller/usRobotTool.cpp
  # )

# IF (USE_VIPER650)
  # LIST(APPEND SRC_CONTROLLER controller/usRobotControllerViper650.cpp)
# ENDIF()

# IF (USE_VIPER850)
  # LIST(APPEND SRC_CONTROLLER controller/usRobotControllerViper850.cpp)
# ENDIF()

# IF (USE_IIWA)
  # LIST(APPEND SRC_CONTROLLER controller/usRobotControllerIIWA.cpp)
# ENDIF()

# SET (SRC_CUDA_NLMEANSFUID
  # cuda/NLMeansFUID/NLMeans_kernel_echo.cu
  # cuda/NLMeansFUID/preprocessing.cu
  # cuda/NLMeansFUID/variance.cpp
  # cuda/NLMeansFUID/usNLMeansFUIDFilter.cu
  # )
  
# SET (SRC_CUDA_GAUSSIANFILTER
  # cuda/cudaGaussianFilter/gaussianFilter_kernel.cu
  # cuda/cudaGaussianFilter/usCudaGaussianFilter.cu
  # )  

# SET (SRC_CUDA_SNAKE_DF
  # cuda/TrackSnakeDF/trackSnakeDFKernel.cu
  # cuda/TrackSnakeDF/trackSnakeDFWithCUDA.cu
  # )

SET (SRC_IO 
  io/usGrabberFile.cpp
  io/usIO_tools.cpp
  )  
  
SET (SRC_DATA
  data/usData.cpp
  data/usDataIo.cpp
  data/usDataPostscan2D.cpp
  data/usDataPostscan3D.cpp
  data/usDataPrescan2D.cpp
  data/usDataPrescan3D.cpp
  # data/usDataRF2D.cpp
  data/usDataRF3D.cpp
)

SET (SRC_GRABBER
  grabber/usGrabber.cpp
)

# IF (USE_ULTRASONIX)
	# SET (SRC_GRABBER ${SRC_GRABBER} grabber/usGrabberUlterius.cpp)
	# SET (SRC_GRABBER ${SRC_GRABBER} grabber/usGrabberUltrasonix.cpp)
# ENDIF()

# IF (USE_V4L2)
	# SET (SRC_GRABBER ${SRC_GRABBER} grabber/usGrabberV4L2.cpp)
# ENDIF()

# SET (SRC_IMAGE
  # image/usImage.cpp
  # image/usImageProcessing.cpp
  # image/usImageMathematics.cpp
  # image/usRectangle.cpp
  # )

# SET (SRC_PROBE
  # probe/usPortaProbeXmlParser.cpp
  # probe/usPortaTools.cpp
# )

# SET(SRC_RF
  # rf/usLogCompressor.cpp
  # rf/usEnvelopeDetector.cpp
# )

# SET(SRC_TRACKING
  # tracking/usTracker2D.cpp
  # tracking/usTrackerDense2D.cpp
# )

# SET (SRC_SCAN_CONVERSION_2D
  # scanConversion2D/usBackScanConverter2D.cpp
  # scanConversion2D/usProbe.cpp
  # scanConversion2D/usScanConversion.cpp
  # scanConversion2D/usScanConverter2D.cpp
# )

# SET(SRC_SCAN_CONVERSION_3D
  # scanConversion3D/StopWatch.cu
  # scanConversion3D/USCudaFunc.cu
  # scanConversion3D/usScanConversion3D.cu
# )

# IF(USTK_HAVE_VTK)
  # LIST(APPEND SRC_SCAN_CONVERSION_3D scanConversion3D/USScanDisp.cpp)
# ENDIF()

# SET (SRC_SNAKE_FOURIER
  # snake/fourier_descriptors/ConfigDF.cpp
  # snake/fourier_descriptors/FiltrageDF.cpp
  # snake/fourier_descriptors/SnakeFourierDescriptor.cpp
  # )

# SET (SRC_SNAKE_OPTIMIZED
  # snake/optimized/Config.cpp
  # snake/optimized/Filtrage.cpp
  # snake/optimized/ImageDoc.cpp
  # snake/optimized/PolaireZone.cpp
  # snake/optimized/nrutil.c
  # snake/optimized/pnmio.c
  # )

# SET (SRC_SNAKE_STANDARD
  # snake/standard/Config.cpp
  # snake/standard/Filtrage.cpp
  # snake/standard/ImageDoc.cpp
  # snake/standard/PolaireZone.cpp
  # snake/standard/nrutil.c
  # snake/standard/pnmio.c
  # )

# SET (SRC_STRADX
  # stradx/usStradxIo.cpp
  # )

# SET (SRC_VOLUME
  # volume/usFrangiFilter.cpp
  # volume/usVolumeProcessing.cpp
# )

# SET (SRC_OPTIMIZATION
  # optimization/usICP_tools.cpp
   # )

SET (SRC_ALL 
  # ${SRC_BI_FD_SNAKE}
  # ${SRC_CUDA_NLMEANSFUID}
  # ${SRC_CUDA_GAUSSIANFILTER}	 
  # ${SRC_CUDA_SNAKE_DF}
  ${SRC_IO}
  ${SRC_DATA}
  ${SRC_GRABBER}
  # ${SRC_IMAGE}
  # ${SRC_TRACKING}
  # ${SRC_SCAN_CONVERSION_2D}
  # ${SRC_SNAKE_FOURIER}
  # ${SRC_VOLUME}
  # ${SRC_OPTIMIZATION}   
)

# IF(USTK_HAVE_CUDA)
  # LIST(APPEND SRC_ALL ${SRC_SCAN_CONVERSION_3D})
# ENDIF()

# IF (USE_ULTRASONIX)
  # LIST(APPEND SRC_ALL ${SRC_PROBE})
# ENDIF()

# IF(BUILD_OPTIMIZED_SNAKE)
  # LIST(APPEND SRC_ALL ${SRC_SNAKE_OPTIMIZED})
# ELSE(BUILD_OPTIMIZED_SNAKE)
  # LIST(APPEND SRC_ALL ${SRC_SNAKE_STANDARD})
# ENDIF(BUILD_OPTIMIZED_SNAKE)

# IF(MODULE_RF)
  # LIST(APPEND SRC_ALL ${SRC_RF})
# ENDIF(MODULE_RF)

# IF(MODULE_ROBOTS)
  # LIST(APPEND SRC_ALL ${SRC_CONTROLLER})
# ENDIF()

# IF(NOT WIN32)
  # LIST(APPEND SRC_ALL ${SRC_STRADX})
# ENDIF()
