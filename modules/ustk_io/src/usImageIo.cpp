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
 * @file usImageIo.cpp
 * @brief Input/output operations between ultrasound data and classical 2D image files.
 */

#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpXmlParser.h>

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usImageSettingsXmlParser.h>

 /**
 * Constructor
 */
usImageIo::usImageIo() {}

/**
* Write 2D rf ultrasound image
*/
bool usImageIo::write(usImageRF2D &rfImage, const std::string filename) {
  return false;
}
  
/**
* Read 2D rf ultrasound image
*/
usImageRF2D usImageIo::readRF2D(const std::string filename) {
  return usImageRF2D();
}

/**
* Write 3D rf ultrasound image
*/
bool usImageIo::write(usImageRF3D &rfImage3D, const std::string filename) {
  return false;
}

/**
* Read 3D rf ultrasound image
*/
usImageRF3D usImageIo::readRF3D(const std::string filename) {
  return usImageRF3D();
}

#ifdef VISP_HAVE_XML2
/**
* Write 2D unsigned char prescan ultrasound image
*/
bool usImageIo::writeXmlPng(usImagePreScan2D<unsigned char> &preScanImage, const std::string filename) {
  try {
    std::string pngFileName = filename + ".png";
    std::string xmlFileName = filename + ".xml";
    vpImageIo::writePNG(preScanImage, pngFileName);
    usImageSettingsXmlParser xmlSettings;
    xmlSettings.setImagePreScanSettings(preScanImage);
    xmlSettings.setImageFileName(pngFileName);
    xmlSettings.save(xmlFileName);
  }
  catch (std::exception e) {
    std::cout << "Error writing postScan image : " << std::endl;
    std::cout << e.what() << std::endl;
    return false;
  }
  return true;
}

/**
* Read 2D unsigned char prescan ultrasound image
*/
usImagePreScan2D<unsigned char> usImageIo::readPreScan2DUCharFromXml(const std::string xmlFilename) {
  usImageSettingsXmlParser xmlSettings;
  try {
    xmlSettings.parse(xmlFilename);
  }
  catch (std::exception e) {
    std::cout << "Error parsing postScan settings file" << std::endl;
    throw e;
  }

  vpImage<unsigned char> image;
  vpImageIo::read(image, xmlSettings.getImageFileName());

  usImagePreScan2D<unsigned char> postScanImage(image, xmlSettings.getImageSettings());
  return postScanImage;
}
#endif //VISP_HAVE_XML2

/**
* Write 3D unsigned char prescan ultrasound image
*/
bool usImageIo::write(usImagePreScan3D<unsigned char> &preScanImage, const std::string filename) {
  return false;
}

/**
* Read 3D unsigned char prescan ultrasound image
*/
usImagePreScan3D<unsigned char> usImageIo::readPreScan3DUChar(const std::string filename) {
  return usImagePreScan3D<unsigned char>();
}

/**
* Write 2D double prescan ultrasound image
*/
bool usImageIo::write(usImagePreScan2D<double> &preScan2DImage, const std::string filename) {
  return  false;
}

/**
* Read 2D double prescan ultrasound image
*/
usImagePreScan2D<double> usImageIo::readPreScan2DDouble(std::string filename) {
  usImagePreScan2D<double> img;
  return img;
}

/**
* Write 3D double prescan ultrasound image
*/
bool usImageIo::write(usImagePreScan3D<double> &preScan3DImage, const std::string filename) {
  return  false;
}

/**
* Read 3D double prescan ultrasound image
*/
usImagePreScan3D<double> usImageIo::readPreScan3DDouble(std::string filename) {
  return  usImagePreScan3D<double>();
}

#ifdef VISP_HAVE_XML2
/**
* Write 2D postscan ultrasound image and settings
* @param postScanImage Image to write
* @param filename The file name without extenstion (same name for png and xml);
*/
bool usImageIo::writeXmlPng(usImagePostScan2D &postScanImage, const std::string filename) {
  try {
    std::string pngFileName = filename + ".png";
    std::string xmlFileName = filename + ".xml";
    vpImageIo::writePNG(postScanImage, pngFileName);
    usImageSettingsXmlParser xmlSettings;
    xmlSettings.setImagePostScanSettings(postScanImage);
    xmlSettings.setImageFileName(pngFileName);
    xmlSettings.save(xmlFileName);
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
* @param xmlFilename The xml file name with .xml extenstion (make sure png file is in the same directory);
*/
usImagePostScan2D usImageIo::readPostScan2DFromXml(const std::string xmlFilename) {
  usImageSettingsXmlParser xmlSettings;
  try {
    xmlSettings.parse(xmlFilename);
  }
  catch(std::exception e) {
    std::cout << "Error parsing postScan settings file" << std::endl;
    throw e;
  }

  vpImage<unsigned char> image;
  vpImageIo::read(image,xmlSettings.getImageFileName());

  usImagePostScan2D postScanImage(image,xmlSettings.getImageSettings());
  return postScanImage;
}
#endif //VISP_HAVE_XML2

/**
* Write 3D postscan ultrasound image and settings
*/
bool usImageIo::write(usImagePostScan3D &postScanImage, const std::string filename) {
  return false;
}

/**
* Read 3D postscan ultrasound image
*/
usImagePostScan3D usImageIo::readPostScan3D() {
  return usImagePostScan3D();
}
