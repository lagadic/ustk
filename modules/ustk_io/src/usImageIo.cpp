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

usImageIo::usHeaderFormatType
usImageIo::getHeaderFormat(const std::string &headerFileName)
{
  std::string ext = usImageIo::getExtension(headerFileName);
  if (ext.compare(".xml") == 0)
    return FORMAT_XML;
  else if (ext.compare(".XML") == 0)
    return FORMAT_XML;
  else if (ext.compare(".mhd") == 0)
    return FORMAT_MHD;
  else if (ext.compare(".MHD") == 0)
    return FORMAT_MHD;
  else
    return HEADER_FORMAT_UNKNOWN;
}

std::string usImageIo::getExtension(const std::string &filename)
{
  // extract the extension
  size_t dot = filename.find_last_of(".");
  std::string ext = filename.substr(dot, filename.size() - 1);
  return ext;
}

/**
* Write 2D rf ultrasound image.
* @param imageRf2D The RF image to write.
* @param headerFileName The header file name to write.
*/
void usImageIo::write(const usImageRF2D<unsigned char> &imageRf2D, const std::string headerFileName) {

}

/**
* Read 2D rf ultrasound image.
* @param [out] imageRf2D The RF image to read.
* @param [in] headerFileName The header file name to read.
*/
void usImageIo::read(usImageRF2D<unsigned char> &imageRf2D, const std::string headerFileName) {

}

/**
* Write 3D rf ultrasound image.
* @param imageRf3D The RF image to write.
* @param headerFileName The header file name to write.
*/
void usImageIo::write(const usImageRF3D<unsigned char> &imageRf3D, const std::string headerFileName) {

}

/**
* Read 3D rf ultrasound image.
* @param [out] imageRf3 The RF image to read.
* @param [in] headerFileName The header file name to read.
*/
void usImageIo::read(usImageRF3D<unsigned char> &imageRf3,const std::string headerFileName) {

}

/**
* Write 2D unsigned char pre-scan ultrasound image.
* @param preScanImage The pre-scan image to write.
* @param headerFileName The header file name to write, with extension.
* @param imageExtension The image extention name to write (ex : ".png").
*/
void usImageIo::write(const usImagePreScan2D<unsigned char> & preScanImage, const std::string headerFileName, const std::string imageExtension) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtension);
#ifdef VISP_HAVE_XML2
    try {
      //writing image
      vpImageIo::write(preScanImage, imageFileName);
      //writing xml
      usImageSettingsXmlParser xmlSettings;
      xmlSettings.setImagePreScanSettings(preScanImage);
      xmlSettings.setImageFileName(imageFileName);
      //write xml
      xmlSettings.save(headerFileName);
    }
    catch (std::exception e) {
      std::cout << "Error writing postScan image : " << std::endl;
      std::cout << e.what() << std::endl;
    }
#else
    throw(vpException(vpException::fatalError,"Requires xml2"));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(".raw");
    //mhd & raw writing
  }
  else {
    throw(vpException(vpException::fatalError, "Unknown extension."));
  }

}

/**
* Read 2D unsigned char pre-scan ultrasound image.
* @param [out] preScanImage The pre-scan image to read.
* @param [in] headerFileName The header file name to read, with extension.
*/
void usImageIo::read(usImagePreScan2D<unsigned char> &preScanImage,const std::string headerFileName)
{
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    //parsing xml file
    usImageSettingsXmlParser xmlSettings;
    try {
      xmlSettings.parse(headerFileName);
    }
    catch (std::exception e) {
      std::cout << "Error parsing postScan settings file" << std::endl;
      throw e;
    }
    vpImageIo::read(preScanImage, xmlSettings.getImageFileName());

    preScanImage.setImageSettings(usImagePreScanSettings(xmlSettings.getImageSettings().getProbeRadius(),
      xmlSettings.getImageSettings().getScanLinePitch(), xmlSettings.getImageSettings().isTransducerConvex(), xmlSettings.getAxialResolution()));
    preScanImage.setAxialResolution(xmlSettings.getAxialResolution());
#else
    throw(vpException(vpException::fatalEtrror, "Requires xml2 library"));
#endif //VISP_HAVE_XML2
  }
  else if (headerFormat == FORMAT_MHD) {
    //mhd reading

  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Write 3D unsigned char pre-scan ultrasound image.
* @param preScanImage The pre-scan image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImagePreScan3D<unsigned char> &preScanImage, const std::string filename) {

}

/**
* Read 3D unsigned char pre-scan ultrasound image.
* @param [out] preScanImage The pre-scan image to read.
* @param [in] filename The image file name to read, with .xml extension.
*/
void usImageIo::read(usImagePreScan3D<unsigned char> &preScanImage,const std::string filename) {

}

/**
* Write 2D double pre-scan ultrasound image.
* @param preScan2DImage The pre-scan image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImagePreScan2D<double> &preScan2DImage, const std::string filename) {

}

/**
* Read 2D double pre-scan ultrasound image.
* @param [out] preScan2D The pre-scan image to read.
* @param [in] filename The image file name to read.
*/
void usImageIo::read(usImagePreScan2D<double> &preScan2D,std::string filename) {

}

/**
* Write 3D double pre-scan ultrasound image.
* @param preScan3DImage The pre-scan image to write.
* @param filename The image file name to write.
*/
void usImageIo::write(const usImagePreScan3D<double> &preScan3DImage, const std::string filename) {

}

/**
* Read 3D double pre-scan ultrasound image.
* @param [out] preScan3DImage The pre-scan image to read.
* @param [in] filename The image file name to read.
*/
void usImageIo::read(usImagePreScan3D<double> &preScan3DImage,std::string filename) {

}

/**
* Write 2D post-scan ultrasound image and settings.
* @param postScanImage Image to write.
* @param headerFileName The header file name with the desired extension.
* @param imageExtension The image extension.
*/
void usImageIo::write(const usImagePostScan2D<unsigned char> &postScanImage, const std::string headerFileName,const std::string imageExtension) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtension);
    try {
      //writing image
      vpImageIo::writePNG(postScanImage, imageFileName);
      //writing xml file using xml parser
      usImageSettingsXmlParser xmlSettings;
      xmlSettings.setImagePostScanSettings(postScanImage);
      xmlSettings.setImageFileName(imageFileName);
      xmlSettings.save(headerFileName);
    }
    catch (std::exception e) {
      std::cout << "Error writing postScan image : " << std::endl;
      std::cout << e.what() << std::endl;
    }
#else
    throw(vpException(vpException::fatalError, "Requires xml2 library"));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    //mhd writing
    //!\\ imageExtension to .raw !!!
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Read 2D post-scan ultrasound image.
* @param [out] postScanImage The post-scan image to read.
* @param [in] headerFileName The header file name with.
*/
void usImageIo::read(usImagePostScan2D<unsigned char> &postScanImage,const std::string headerFileName) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    usImageSettingsXmlParser xmlSettings;
    try {
      xmlSettings.parse(headerFileName);
    }
    catch (std::exception e) {
      std::cout << "Error parsing postScan settings file" << std::endl;
      throw e;
    }

    vpImage<unsigned char> image;
    vpImageIo::read(postScanImage, xmlSettings.getImageFileName());

    postScanImage.setImageSettings(usImagePostScanSettings(xmlSettings.getImageSettings(), xmlSettings.getHeightResolution(), xmlSettings.getWidthResolution()));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    //mhd reading

  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Write 3D post-scan ultrasound image and settings
* @param postScanImage Image to write.
* @param mhdFilename The header file name.
* @param imageExtension The image file extension.
*/
void usImageIo::write(const usImagePostScan3D<unsigned char> &postScanImage, const std::string headerFileName, const std::string imageExtension)
{
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
   
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
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
    header.MHDFileName = headerFileName;
    header.rawFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtension);
    header.isTransducerConvex = postScanImage.isTransducerConvex();
    header.isMotorRotating = postScanImage.isMotorRotating();
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
    rawParser.write(postScanImage, header.rawFileName);
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Read 3D post-scan ultrasound image
* @param [out] postScanImage The post-scan image to read.
* @param [in] mhdFileName The mhd file name with .mhd extenstion.
*/
void usImageIo::read(usImagePostScan3D<unsigned char> &postScanImage,std::string headerFileName) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2

#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    //header parsing
    usMetaHeaderParser mhdParser;
    mhdParser.read(headerFileName);
    if (mhdParser.getImageType() != usMetaHeaderParser::POSTSCAN_3D && mhdParser.getImageType() != usMetaHeaderParser::NOT_SET) {
      throw(vpException(vpException::badValue,"Reading a non post-scan 3D image!"));
    }
    if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
      throw(vpException(vpException::badValue,"Reading a non unsigned char image!"));
    }
    //resizing image in memory
    postScanImage.resize(mhdParser.getImageSizeX(),mhdParser.getImageSizeY(),mhdParser.getImageSizeZ());

    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

    usImagePostScan3DSettings settings;
    settings.setProbeRadius(mhdHeader.probeRadius);
    settings.setScanLinePitch(mhdHeader.scanLinePitch);
    settings.setProbeConvexity(mhdHeader.isTransducerConvex);
    settings.setMotorRadius(mhdHeader.motorRadius);
    settings.setFramePitch(mhdHeader.framePitch);
    settings.setMotorConvexity(mhdHeader.isMotorRotating);
    settings.setWidthResolution(mhdParser.getWidthResolution());
    settings.setHeightResolution(mhdParser.getHeightResolution());
    postScanImage.setImageSettings(settings);

    //data parsing
    usRawFileParser rawParser;
    rawParser.read(postScanImage,mhdParser.getRawFileName());
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}
