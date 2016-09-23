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
 * @file usImageIo.h
 * @brief Input/output operations between ultrasound data and image files and their associated header (containing usImageSettings info).
 */

#ifndef US_IMAGE_IO_H
#define US_IMAGE_IO_H

#include <string>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_io/usMetaHeaderParser.h>


/**
 * @class usImageIo
 * @brief Input/output operations between ultrasound data and image files and their associated header (containing usImageSettings info).
 */
class VISP_EXPORT usImageIo
{
public:

  usImageIo();

  //RF
  bool write(usImageRF2D &rfImage, const std::string filename);
  usImageRF2D readRF2D(const std::string filename);
  bool write(usImageRF3D &rfImage3D, const std::string filename);
  usImageRF3D readRF3D(const std::string filename);

  //PreScan-unsigned char
#ifdef VISP_HAVE_XML2
  bool writeXmlPng(usImagePreScan2D<unsigned char> &preScanImage, const std::string filename);
  usImagePreScan2D<unsigned char> readPreScan2DUCharFromXml(const std::string filename);
#endif //VISP_HAVE_XML2
  bool write(usImagePreScan3D<unsigned char> &preScanImage3D, const std::string filename);
  usImagePreScan3D<unsigned char> readPreScan3DUChar(const std::string filename);

  //PreScan-double
  bool write(usImagePreScan2D<double> &preScanImage, const std::string filename);
  usImagePreScan2D<double> readPreScan2DDouble(const std::string filename);
  bool write(usImagePreScan3D<double> &preScanImage3D, const std::string filename);
  usImagePreScan3D<double> readPreScan3DDouble(const std::string filename);

  //postScan
#ifdef VISP_HAVE_XML2
  bool writeXmlPng(usImagePostScan2D &postScanImage, const std::string filename);
  usImagePostScan2D readPostScan2DFromXml(const std::string filename);
#endif //VISP_HAVE_XML2
  bool write(usImagePostScan3D &postScanImage3D, const std::string filename);
  usImagePostScan3D readPostScan3D(std::string mhdFileName);
};

#endif //US_IMAGE_IO_H
