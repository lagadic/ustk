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
#include <visp3/ustk_core/usImagePostScan3DSettings.h>
#include <visp3/ustk_core/usImagePreScanSettings.h>
#include <visp3/ustk_io/usRawFileParser.h>

/**
* Write 2D rf ultrasound image.
* @param imageRf2D The RF image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImageRF2D<unsigned char> &imageRf2D, const std::string filename) {

}

/**
* Read 2D rf ultrasound image.
* @param [out] imageRf2D The RF image to read.
* @param [in] filename The image file name to read.
*/
void usImageIo::read(usImageRF2D<unsigned char> &imageRf2D, const std::string filename) {

}

/**
* Write 3D rf ultrasound image.
* @param imageRf3D The RF image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImageRF3D<unsigned char> &imageRf3D, const std::string filename) {

}

/**
* Read 3D rf ultrasound image.
* @param [out] imageRf3 The RF image to read.
* @param [in] filename The image file name to read.
*/
void usImageIo::read(usImageRF3D<unsigned char> &imageRf3,const std::string filename) {

}

#ifdef VISP_HAVE_XML2
/**
* Write 2D unsigned char prescan ultrasound image.
* @param preScanImage The prescan image to write.
* @param imageFileName The image file name to write, with extension.
*/
void usImageIo::writeXml(const usImagePreScan2D<unsigned char> & preScanImage, const std::string imageFileName) {
  try {
    //writing image
    vpImageIo::write(preScanImage, imageFileName);
    //writing xml
    usImageSettingsXmlParser xmlSettings;
    xmlSettings.setImagePreScanSettings(preScanImage);
    xmlSettings.setImageFileName(imageFileName);
    //get xml filename from imageFileName
    std::vector<std::string> splittedFileName = vpIoTools::splitChain(imageFileName, ".");
    std::string xmlFileName = splittedFileName[0] + ".xml";
    xmlSettings.save(xmlFileName);
  }
  catch (std::exception e) {
    std::cout << "Error writing postScan image : " << std::endl;
    std::cout << e.what() << std::endl;
  }
}

/**
* Read 2D unsigned char prescan ultrasound image.
* @param [out] preScanImage The prescan image to read.
* @param [in] xmlFileName The xml file name to read, with .xml extension.
*/
void usImageIo::readXml(usImagePreScan2D<unsigned char> &preScanImage,const std::string xmlFileName)
{
  //parsing xml file
  usImageSettingsXmlParser xmlSettings;
  try {
    xmlSettings.parse(xmlFileName);
  }
  catch (std::exception e) {
    std::cout << "Error parsing postScan settings file" << std::endl;
    throw e;
  }
  vpImageIo::read(preScanImage, xmlSettings.getImageFileName());

  preScanImage.setImageSettings(usImagePreScanSettings(xmlSettings.getImageSettings().getProbeRadius(), xmlSettings.getImageSettings().getScanLinePitch(),xmlSettings.getImageSettings().isProbeConvex(), xmlSettings.getAxialResolution()));
  preScanImage.setAxialResolution(xmlSettings.getAxialResolution());
}
#endif //VISP_HAVE_XML2

/**
* Write 3D unsigned char prescan ultrasound image.
* @param preScanImage The prescan image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImagePreScan3D<unsigned char> &preScanImage, const std::string filename) {

}

/**
* Read 3D unsigned char prescan ultrasound image.
* @param [out] preScanImage The prescan image to read.
* @param [in] filename The image file name to read, with .xml extension.
*/
void usImageIo::read(usImagePreScan3D<unsigned char> &preScanImage,const std::string filename) {

}

/**
* Write 2D double prescan ultrasound image.
* @param preScan2DImage The prescan image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImagePreScan2D<double> &preScan2DImage, const std::string filename) {

}

/**
* Read 2D double prescan ultrasound image.
* @param [out] preScan2D The prescan image to read.
* @param [in] filename The image file name to read.
*/
void usImageIo::read(usImagePreScan2D<double> &preScan2D,std::string filename) {

}

/**
* Write 3D double prescan ultrasound image.
* @param preScan3DImage The prescan image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImagePreScan3D<double> &preScan3DImage, const std::string filename) {

}

/**
* Read 3D double prescan ultrasound image.
* @param [out] preScan3DImage The prescan image to read.
* @param [in] filename The image file name to read.
*/
void usImageIo::read(usImagePreScan3D<double> &preScan3DImage,std::string filename) {

}

#ifdef VISP_HAVE_XML2
/**
* Write 2D postscan ultrasound image and settings.
* @param postScanImage Image to write.
* @param imageFilename The image file name with the desired extenstion.
*/
void usImageIo::writeXml(const usImagePostScan2D<unsigned char> &postScanImage, const std::string imageFilename) {
  try {
    //writing image
    vpImageIo::writePNG(postScanImage, imageFilename);
    //geting xml file name
    std::vector<std::string> splittedFileName = vpIoTools::splitChain(imageFilename, ".");
    std::string xmlFileName = splittedFileName[0] + ".xml";
    //writing xml file using xml parser
    usImageSettingsXmlParser xmlSettings;
    xmlSettings.setImagePostScanSettings(postScanImage);
    xmlSettings.setImageFileName(imageFilename);
    xmlSettings.save(xmlFileName);
  }
  catch (std::exception e) {
    std::cout << "Error writing postScan image : " << std::endl;
    std::cout << e.what() << std::endl;
  }
}

/**
* Read 2D postscan ultrasound image.
* @param [out] postScanImage The postscan image to read.
* @param [in] xmlFilename The xml file name with .xml extenstion.
*/
void usImageIo::readXml(usImagePostScan2D<unsigned char> &postScanImage,const std::string xmlFilename) {
  usImageSettingsXmlParser xmlSettings;
  try {
    xmlSettings.parse(xmlFilename);
  }
  catch(std::exception e) {
    std::cout << "Error parsing postScan settings file" << std::endl;
    throw e;
  }

  vpImage<unsigned char> image;
  vpImageIo::read(postScanImage,xmlSettings.getImageFileName());

  postScanImage.setImageSettings(usImagePostScanSettings(xmlSettings.getImageSettings(), xmlSettings.getHeightResolution(),xmlSettings.getWidthResolution()));
}
#endif //VISP_HAVE_XML2

/**
* Write 3D postscan ultrasound image and settings
* @param postScanImage Image to write.
* @param filename The image file name with the desired extenstion.
*/
void usImageIo::write(const usImagePostScan3D<unsigned char> &postScanImage, const std::string filename)
{
  //filling header
  usMetaHeaderParser::MHDHeader header;
  header.numberOfDimensions = 3;
  header.elementType = usMetaHeaderParser::MET_UCHAR;
  header.imageType = usMetaHeaderParser::POSTSCAN_3D;
  header.elementSpacing[0] = postScanImage.getElementSpacingX();
  header.elementSpacing[1] = postScanImage.getElementSpacingY();
  header.elementSpacing[2] = postScanImage.getElementSpacingZ();
  header.dim[0] = postScanImage.getANumber();
  header.dim[1] = postScanImage.getLineNumber();
  header.dim[2] = postScanImage.getFrameNumber();
  header.msb = false;
  header.MHDFileName = filename + ".mhd";
  header.rawFileName = filename + ".raw";
  header.isProbeConvex = postScanImage.isProbeConvex();
  header.isMotorConvex = postScanImage.isMotorConvex();
  header.probeRadius = postScanImage.getProbeRadius();
  header.scanLinePitch = postScanImage.getScanLinePitch();
  header.motorRadius = postScanImage.getMotorRadius();
  header.framePitch = postScanImage.getFramePitch();
  //writing in file
  usMetaHeaderParser mhdParser;
  mhdParser.setMHDHeader(header);
  mhdParser.setHeightResolution(postScanImage.getHeightResolution());
  mhdParser.setWidthResolution(postScanImage.getWidthResolution());
  mhdParser.parse();

  //filling raw
  usRawFileParser rawParser;
  rawParser.write(postScanImage,header.rawFileName);
}

/**
* Read 3D postscan ultrasound image
* @param [out] postScanImage The postscan image to read.
* @param [in] mhdFileName The mhd file name with .mhd extenstion.
*/
void usImageIo::read(usImagePostScan3D<unsigned char> &postScanImage,std::string mhdFileName) {

  //header parsing
  usMetaHeaderParser mhdParser;
  mhdParser.read(mhdFileName);
  if (mhdParser.getImageType() != usMetaHeaderParser::POSTSCAN_3D) {
    throw(vpException(vpException::badValue,"Reading a non postscan3d image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue,"Reading a non unisgned char image!"));
  }
  //resizing image in memory
  postScanImage.resize(mhdParser.getImageSizeX(),mhdParser.getImageSizeY(),mhdParser.getImageSizeZ());

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

  usImagePostScan3DSettings settings;
  settings.setProbeRadius(mhdHeader.probeRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setProbeConvexity(mhdHeader.isProbeConvex);
  settings.setMotorRadius(mhdHeader.motorRadius);
  settings.setFramePitch(mhdHeader.framePitch);
  settings.setMotorConvexity(mhdHeader.isMotorConvex);
  settings.setWidthResolution(mhdParser.getWidthResolution());
  settings.setHeightResolution(mhdParser.getHeightResolution());
  postScanImage.setImageSettings(settings);

  //data parsing
  usRawFileParser rawParser;
  rawParser.read(postScanImage,mhdParser.getRawFileName());
}
