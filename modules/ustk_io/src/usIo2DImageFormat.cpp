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
 * @file usIo2DimageFormat.cpp
 * @brief Input/output operations between ultrasound data and classical 2D image files.
 */

#include <visp3/io/vpImageIo.h>

#include <visp3/ustk_io/usIo2DImageFormat.h>

 /**
 * Constructor
 */
usIo2DImageFormat::usIo2DImageFormat() {}

/**
* Write 2D rf ultrasound image
*/
bool usIo2DImageFormat::write(usImageRF2D &rfImage, const std::string filename) {
  return false;
}
  
/**
* Read 2D rf ultrasound image
*/
usImageRF2D usIo2DImageFormat::readRF(const std::string filename) {
  return usImageRF2D();
}

/**
* Write 2D prescan ultrasound image
*/
bool usIo2DImageFormat::write(usImagePreScan2D<unsigned char> &preScanImage, const std::string filename) {
  return false;
}

/**
* Read 2D prescan ultrasound image
*/
usImagePreScan2D<unsigned char> readPreScanUChar(const std::string filename) {
  return usImagePreScan2D<unsigned char>();
}

/**
* Write 2D prescan ultrasound image
*/
bool usIo2DImageFormat::write(usImagePreScan2D<double> &preScanImage, const std::string filename) {
  return  false;
}

/**
* Read 2D prescan ultrasound image
*/
usImagePreScan2D<double> usIo2DImageFormat::readPreScanDouble(std::string filename) {
  return  usImagePreScan2D<double>();
}

/**
* Write 2D postscan ultrasound image
*/
bool usIo2DImageFormat::write(usImagePostScan2D &postScanImage, const std::string filename) {
  try {
    vpImageIo::writePNG(postScanImage, filename);
  }
  catch (std::exception e) {
    std::cout << "Error writing postScan image : " << std::endl;
    std::cout << e.what() << std::endl;
    return false;
  }
  
  return true;
}

/**
* Read 2D postscan ultrasound image
*/
usImagePostScan2D usIo2DImageFormat::readPostScan(const std::string filename) {
  return
}
