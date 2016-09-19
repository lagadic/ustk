/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usIo2DimageFormat.h
 * @brief Input/output operations between ultrasound data and classical 2D image files.
 */

#ifndef US_IO_2D_IMAGE_FORMAT_H
#define US_IO_2D_IMAGE_FORMAT_H

#include <string>

//#include <UsTk/usTkConfig.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImageRF2D.h>

/**
 * @class usIo2DImageFormat
 * @brief Input/output operations between ultrasound data and classical 2D image files.
 */
class VISP_EXPORT usIo2DImageFormat
{
  /**
   * Write 2D prescan ultrasound data as MHD.
   * @param[in] data The ultrasound data.
   * @param[in] filename File name.
   */
public:

  usIo2DImageFormat();

  //RF
  bool write(usImageRF2D &rfImage, const std::string filename);
  usImageRF2D readRF(const std::string filename);

  //PreScan
  bool write(usImagePreScan2D<unsigned char> &preScanImage, const std::string filename);
  usImagePreScan2D<unsigned char> readPreScanUChar(const std::string filename);
  bool write(usImagePreScan2D<double> &preScanImage, const std::string filename);
  usImagePreScan2D<double> readPreScanDouble(const std::string filename);

  //postScan
  bool write(usImagePostScan2D &postScanImage, const std::string filename);
  usImagePostScan2D readPostScan(const std::string filename);

};

#endif //US_IO_2D_IMAGE_FORMAT_H
