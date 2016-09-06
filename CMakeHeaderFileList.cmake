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

# Set HEADER_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update HEADER_ALL variable if you add/remove a 
# HEADER_subdir variable
#
# If you add/remove a directory, modify here

# SET (HEADER_BI_FD_SNAKE
  # snake/usBiFDSnake/usBiSnake.h
  # )

# SET (HEADER_CONTROLLER
  # controller/usRobotController.h
  # controller/usWorkingPoint.h
  # controller/usRobotTool.h
  # )

# IF (USE_VIPER650)
  # SET (HEADER_CONTROLLER ${HEADER_CONTROLLER} controller/usRobotControllerViper650.h)
# ENDIF()

# IF (USE_VIPER850)
  # SET (HEADER_CONTROLLER ${HEADER_CONTROLLER} controller/usRobotControllerViper850.h)
# ENDIF()

# IF (USE_IIWA)
  # SET(HEADER_CONTROLLER ${HEADER_CONTROLLER} controller/usRobotControllerIIWA.h)
# ENDIF()

# SET (HEADER_CUDA_NLMEANSFUID
  # cuda/NLMeansFUID/NLMeans_kernel_echo.h
  # cuda/NLMeansFUID/preprocessing.h
  # cuda/NLMeansFUID/variance.h
  # cuda/NLMeansFUID/usNLMeansFUIDFilter.h
  # )

# SET (HEADER_CUDA_GAUSSIANFILTER
  # cuda/cudaGaussianFilter/gaussianFilter_kernel.h
  # cuda/cudaGaussianFilter/usCudaGaussianFilter.h
  # )

# SET (HEADER_CUDA_SNAKE_DF
  # cuda/TrackSnakeDF/configCUDA.h
  # cuda/TrackSnakeDF/trackSnakeDFKernel.h
  # cuda/TrackSnakeDF/trackSnakeDFWithCUDA.h
  # ) 

SET (HEADER_IO 
  io/usGrabberFile.h
  io/usVolumeIo.h
  io/usIO_tools.h
  )
  
SET (HEADER_DATA
  data/usData.h
  data/usDataIo.h
  data/usDataPostscan2D.h
  data/usDataPostscan3D.h
  data/usDataPrescan2D.h
  data/usDataPrescan3D.h
  # data/usDataRF2D.h
  data/usDataRF3D.h
)

SET (HEADER_GRABBER
  grabber/usGrabber.h
  grabber/usGrabberException.h
  )

# IF (USE_ULTRASONIX)
	# SET (HEADER_GRABBER ${HEADER_GRABBER} grabber/usGrabberUlterius.h)
	# SET (HEADER_GRABBER ${HEADER_GRABBER} grabber/usGrabberUltrasonix.h)
# ENDIF()

# IF (USE_V4L2)
	# SET (HEADER_GRABBER ${HEADER_GRABBER} grabber/usGrabberV4L2.h)
# ENDIF()

# SET (HEADER_IMAGE
  # image/usImage.h
  # image/usImageProcessing.h
  # image/usImageMathematics.h
  # image/usRectangle.h
  # )

# SET (HEADER_PROBE
  # probe/porta_def.h
  # probe/usPortaProbeXmlParser.h
  # probe/usPortaTools.h
# )

# SET(HEADER_RF
  # rf/usEnvelopeDetector.h
  # rf/usLogCompressor.h
# )

# SET(HEADER_TRACKING
  # tracking/usTracker2D.h
  # tracking/usTrackerDense2D.h
# )

# SET (HEADER_SCAN_CONVERSION_2D
  # scanConversion2D/usBackScanConverter2D.h
  # scanConversion2D/usProbe.h
  # scanConversion2D/usScanConversion.h
  # scanConversion2D/usScanConverter2D.h
# )

# SET(HEADER_SCAN_CONVERSION_3D
  # scanConversion3D/StopWatch.h
  # scanConversion3D/USCudaFunc.h
  # scanConversion3D/usScanConversion3D.h
# )

# IF(USTK_HAVE_VTK)
  # LIST(APPEND HEADER_SCAN_CONVERSION_3D scanConversion3D/USScanDisp.h)
# ENDIF()

# SET (HEADER_SNAKE_OPTIMIZED
  # snake/optimized/Config.h
  # snake/optimized/Filtrage.h
  # snake/optimized/ImageDoc.h
  # snake/optimized/PolaireZone.h
  # snake/optimized/all.h
  # snake/optimized/constantes.h
  # snake/optimized/nrutil.h
  # snake/optimized/pnmio.h
  # )

# SET (HEADER_SNAKE_STANDARD
  # snake/standard/Config.h
  # snake/standard/Filtrage.h
  # snake/standard/ImageDoc.h
  # snake/standard/PolaireZone.h
  # snake/standard/all.h
  # snake/standard/constantes.h
  # snake/standard/nrutil.h
  # snake/standard/pnmio.h
  # )

# SET (HEADER_SNAKE_FOURIER
  # snake/fourier_descriptors/ConfigDF.h
  # snake/fourier_descriptors/FiltrageDF.h
  # snake/fourier_descriptors/SnakeFourierDescriptor.h
  # )

# SET (HEADER_STRADX
  # stradx/usStradxIo.h
  # )

SET (HEADER_VOLUME
  volume/usVolume.h
  volume/usVolumeTools.h
  # volume/usFrangiFilter.h
  # volume/usVolumeProcessing.h
)

# SET (HEADER_OPTIMIZATION
  # optimization/usICP_tools.h
   # )

SET (HEADER_ALL 
  # ${HEADER_BI_FD_SNAKE}
  # ${HEADER_CUDA_GAUSSIANFILTER}
  # ${HEADER_CUDA_NLMEANSFUID}
  # ${HEADER_CUDA_SNAKE_DF}
  ${HEADER_IO}
  ${HEADER_DATA}
  ${HEADER_GRABBER}
  # ${HEADER_IMAGE}
  # ${HEADER_TRACKING}
  # ${HEADER_SCAN_CONVERSION_2D}
  # ${HEADER_SNAKE_FOURIER}
  ${HEADER_VOLUME}
  # ${HEADER_OPTIMIZATION}
  )

# IF(USTK_HAVE_CUDA)
  # LIST(APPEND HEADER_ALL ${HEADER_SCAN_CONVERSION_3D})
# ENDIF()

# IF (USE_ULTRASONIX)
  # LIST(APPEND HEADER_ALL ${HEADER_PROBE})
# ENDIF()

# IF(BUILD_OPTIMIZED_SNAKE)
  # LIST(APPEND HEADER_ALL ${HEADER_SNAKE_OPTIMIZED})
# ELSE(BUILD_OPTIMIZED_SNAKE)
  # LIST(APPEND HEADER_ALL ${HEADER_SNAKE_STANDARD})
# ENDIF(BUILD_OPTIMIZED_SNAKE)

# IF(MODULE_RF)
  # LIST(APPEND HEADER_ALL ${HEADER_RF})
# ENDIF(MODULE_RF)

# IF(MODULE_ROBOTS)
  # LIST(APPEND HEADER_ALL ${HEADER_CONTROLLER})
# ENDIF()

# IF(NOT WIN32)
  # SET (HEADER_ALL ${HEADER_ALL} ${HEADER_STRADX})
# ENDIF()
