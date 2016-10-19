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
#include <visp3/ustk_core/usMotorSettings.h>
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
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    write(imageRf2D, headerFileName, ".png");
  }
  else if (headerFormat == FORMAT_MHD) {
    write(imageRf2D, headerFileName, ".raw");
  }
}

/**
* Write 2D rf ultrasound image.
* @param imageRf2D The RF image to write.
* @param headerFileName The header file name to write.
* @param imageExtesion2D The 2D image extension.
*/
void usImageIo::write(const usImageRF2D<unsigned char> &imageRf2D, const std::string headerFileName, const std::string imageExtesion2D) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtesion2D);
#ifdef VISP_HAVE_XML2
    try {
      //writing image
      vpImageIo::write(imageRf2D, imageFileName);
      //writing xml
      usImageSettingsXmlParser xmlSettings;
      xmlSettings.setImageType(usImageSettingsXmlParser::IMAGE_TYPE_RF);
      xmlSettings.setImagePreScanSettings(imageRf2D);
      //just writing the image file name without parents directories (header and image are in the same directory).
      imageFileName = vpIoTools::getName(imageFileName);
      xmlSettings.setImageFileName(imageFileName);
      //write xml
      xmlSettings.save(headerFileName);
    }
    catch (std::exception e) {
      std::cout << "Error writing rf image : " << std::endl;
      std::cout << e.what() << std::endl;
    }
#else
    throw(vpException(vpException::fatalError, "Requires xml2"));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    if (imageExtesion2D != ".raw") {
      throw(vpException(vpException::fatalError, "mhd files goes with .raw image extension"));
    }

    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(".raw");
    //filling header
    usMetaHeaderParser::MHDHeader header;
    header.numberOfDimensions = 2;
    header.elementType = usMetaHeaderParser::MET_UCHAR;
    header.imageType = usMetaHeaderParser::RF_2D;
    header.elementSpacing[0] = 1;
    header.elementSpacing[1] = 1;
    header.dim[0] = imageRf2D.getRFSampleNumber();
    header.dim[1] = imageRf2D.getLineNumber();
    header.msb = false;
    header.MHDFileName = headerFileName;
    //remove full path for image file name (located in the same directory as the mhd
    header.rawFileName = vpIoTools::getName(imageFileName);
    header.isTransducerConvex = imageRf2D.isTransducerConvex();
    header.probeRadius = imageRf2D.getProbeRadius();
    header.scanLinePitch = imageRf2D.getScanLinePitch();
    //writing in file
    usMetaHeaderParser mhdParser;
    mhdParser.setMHDHeader(header);
    mhdParser.setAxialResolution(imageRf2D.getAxialResolution());
    mhdParser.parse();

    //filling raw
    usRawFileParser rawParser;
    rawParser.write(imageRf2D, imageFileName);
  }
  else {
    throw(vpException(vpException::fatalError, "Unknown extension."));
  }
}

/**
* Read 2D rf ultrasound image.
* @param [out] imageRf2D The RF image to read.
* @param [in] headerFileName The header file name to read.
*/
void usImageIo::read(usImageRF2D<unsigned char> &imageRf2D, const std::string headerFileName) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    //parsing xml file
    std::cout << headerFileName << std::endl;
    usImageSettingsXmlParser xmlSettings;
    try {
      xmlSettings.parse(headerFileName);
    }
    catch (std::exception e) {
      std::cout << "Error parsing rf settings file" << std::endl;
      throw e;
    }
    std::string fullImageFileName = vpIoTools::getParent(headerFileName) + vpIoTools::path("/") + xmlSettings.getImageFileName();
    vpImageIo::read(imageRf2D, fullImageFileName);

    imageRf2D.setImageSettings(usImagePreScanSettings(xmlSettings.getImagePreScanSettings().getProbeRadius(),
      xmlSettings.getImagePreScanSettings().getScanLinePitch(), xmlSettings.getImagePreScanSettings().isTransducerConvex(), xmlSettings.getImagePreScanSettings().getAxialResolution()));
#else
    throw(vpException(vpException::fatalEtrror, "Requires xml2 library"));
#endif //VISP_HAVE_XML2
  }
  else if (headerFormat == FORMAT_MHD) {
    //header parsing
    usMetaHeaderParser mhdParser;
    mhdParser.read(headerFileName);
    if (mhdParser.getImageType() != usMetaHeaderParser::RF_2D && mhdParser.getImageType() != usMetaHeaderParser::NOT_SET) {
      throw(vpException(vpException::badValue, "Reading a non rf 2D image!"));
    }
    if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
      throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
    }

    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

    usImagePreScanSettings settings;
    settings.setProbeRadius(mhdHeader.probeRadius);
    settings.setScanLinePitch(mhdHeader.scanLinePitch);
    settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
    settings.setAxialResolution(mhdParser.getAxialResolution());
    imageRf2D.setImageSettings(settings);

    //resizing image in memory
    imageRf2D.resize(mhdParser.getANumber(), mhdParser.getLineNumber());

    //data parsing
    usRawFileParser rawParser;
    rawParser.read(imageRf2D, mhdParser.getRawFileName());
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Write 3D rf ultrasound image.
* @param imageRf3D The RF image to write.
* @param headerFileName The header file name to write.
*/
void usImageIo::write(const usImageRF3D<unsigned char> &imageRf3D, const std::string headerFileName) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    write(imageRf3D, headerFileName, ".png");
  }
  else if (headerFormat == FORMAT_MHD) {
    write(imageRf3D, headerFileName, ".raw");
  }
}

/**
* Write 3D rf ultrasound image.
* @param imageRf3D The RF image to write.
* @param headerFileName The header file name to write.
* @param imageExtesion2D The 2D image extension.
*/
void usImageIo::write(const usImageRF3D<unsigned char> &imageRf3D, const std::string headerFileName, const std::string imageExtesion2D) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtesion2D);
#ifdef VISP_HAVE_XML2
    //case of a set of successive 2D frames
#else
    throw(vpException(vpException::fatalError, "Requires xml2"));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    if (imageExtesion2D != ".raw") {
      throw(vpException(vpException::fatalError, "mhd files goes with .raw image extension"));
    }
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(".raw");
    //filling header
    usMetaHeaderParser::MHDHeader header;
    header.numberOfDimensions = 3;
    header.elementType = usMetaHeaderParser::MET_UCHAR;
    header.imageType = usMetaHeaderParser::RF_3D;
    header.elementSpacing[0] = 1;
    header.elementSpacing[1] = 1;
    header.elementSpacing[2] = 1;
    header.dim[0] = imageRf3D.getDimX();
    header.dim[1] = imageRf3D.getDimY();
    header.dim[1] = imageRf3D.getDimZ();
    header.dim[2] = 1;
    header.msb = false;
    header.MHDFileName = headerFileName;
    //remove full path for image file name (located in the same directory as the mhd
    header.rawFileName = vpIoTools::getName(imageFileName);
    header.isTransducerConvex = imageRf3D.isTransducerConvex();
    header.probeRadius = imageRf3D.getProbeRadius();
    header.scanLinePitch = imageRf3D.getScanLinePitch();
    header.motorType = imageRf3D.getMotorType();
    header.motorRadius = imageRf3D.getMotorRadius();
    header.framePitch = imageRf3D.getFramePitch();
    //writing in file
    usMetaHeaderParser mhdParser;
    mhdParser.setMHDHeader(header);
    mhdParser.setAxialResolution(imageRf3D.getAxialResolution());
    mhdParser.parse();

    //filling raw
    usRawFileParser rawParser;
    rawParser.write(imageRf3D, imageFileName);
  }
  else {
    throw(vpException(vpException::fatalError, "Unknown extension."));
  }
}

/**
* Read 3D rf ultrasound image.
* @param [out] imageRf3 The RF image to read.
* @param [in] headerFileName The header file name to read.
*/
void usImageIo::read(usImageRF3D<unsigned char> &imageRf3,const std::string headerFileName) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    //case of a set of sucessive 2D frames
#else
    throw(vpException(vpException::fatalEtrror, "Requires xml2 library"));
#endif //VISP_HAVE_XML2
  }
  else if (headerFormat == FORMAT_MHD) {
    //header parsing
    usMetaHeaderParser mhdParser;
    mhdParser.read(headerFileName);
    if (mhdParser.getImageType() != usMetaHeaderParser::RF_3D && mhdParser.getImageType() != usMetaHeaderParser::NOT_SET) {
      throw(vpException(vpException::badValue, "Reading a non rf 3D image!"));
    }
    if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
      throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
    }
    //resizing image in memory
    imageRf3.resize(mhdParser.getANumber(), mhdParser.getLineNumber(), mhdParser.getFrameNumber());

    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

    usImagePreScanSettings settings;
    settings.setProbeRadius(mhdHeader.probeRadius);
    settings.setScanLinePitch(mhdHeader.scanLinePitch);
    settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
    settings.setAxialResolution(mhdParser.getAxialResolution());
    imageRf3.setImageSettings(settings);

    usMotorSettings motorSettings;
    motorSettings.setMotorRadius(mhdHeader.motorRadius);
    motorSettings.setFramePitch(mhdHeader.framePitch);
    motorSettings.setMotorType(mhdHeader.motorType);
    imageRf3.setMotorSettings(motorSettings);

    //data parsing
    usRawFileParser rawParser;
    rawParser.read(imageRf3, mhdParser.getRawFileName());
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Write 2D unsigned char pre-scan ultrasound image.
* @param preScanImage The pre-scan image to write.
* @param headerFileName The header file name to write, with extension.
*/
void usImageIo::write(const usImagePreScan2D<unsigned char> & preScanImage, const std::string headerFileName) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    write(preScanImage, headerFileName, ".png");
  }
  else if (headerFormat == FORMAT_MHD) {
    write(preScanImage, headerFileName, ".raw");
  }
}

/**
* Write 2D unsigned char pre-scan ultrasound image.
* @param preScanImage The pre-scan image to write.
* @param headerFileName The header file name to write, with extension.
* @param imageExtesion2D The image extention name to write (ex : ".png").
*/
void usImageIo::write(const usImagePreScan2D<unsigned char> & preScanImage, const std::string headerFileName, const std::string imageExtesion2D) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtesion2D);
#ifdef VISP_HAVE_XML2
    try {
      //writing image
      vpImageIo::write(preScanImage, imageFileName);
      //writing xml
      usImageSettingsXmlParser xmlSettings;
      xmlSettings.setImageType(usImageSettingsXmlParser::IMAGE_TYPE_PRESCAN);
      xmlSettings.setImagePreScanSettings(preScanImage);
      //just writing the image file name without parents directories (header and image are in the same directory).
      imageFileName = vpIoTools::getName(imageFileName);
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
    if (imageExtesion2D != ".raw") {
      throw(vpException(vpException::fatalError, "mhd files goes with .raw image extension"));
    }
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(".raw");
    //filling header
    usMetaHeaderParser::MHDHeader header;
    header.numberOfDimensions = 2;
    header.elementType = usMetaHeaderParser::MET_UCHAR;
    header.imageType = usMetaHeaderParser::PRESCAN_2D;
    header.elementSpacing[0] = 1;
    header.elementSpacing[1] = 1;
    header.dim[0] = preScanImage.getLineNumber();
    header.dim[1] = preScanImage.getBModeSampleNumber();
    header.msb = false;
    header.MHDFileName = headerFileName;
    //remove full path for image file name (located in the same directory as the mhd
    header.rawFileName = vpIoTools::getName(imageFileName);
    header.isTransducerConvex = preScanImage.isTransducerConvex();
    header.probeRadius = preScanImage.getProbeRadius();
    header.scanLinePitch = preScanImage.getScanLinePitch();
    //writing in file
    usMetaHeaderParser mhdParser;
    mhdParser.setMHDHeader(header);
    mhdParser.setAxialResolution(preScanImage.getAxialResolution());
    mhdParser.parse();

    //filling raw
    usRawFileParser rawParser;
    rawParser.write(preScanImage, imageFileName);
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
      std::cout << "Error parsing pre-scan settings file" << std::endl;
      throw e;
    }
    std::string fullImageFileName = vpIoTools::getParent(headerFileName) + vpIoTools::path("/") + xmlSettings.getImageFileName();
    vpImageIo::read(preScanImage, fullImageFileName);

    preScanImage.setImageSettings(usImagePreScanSettings(xmlSettings.getImagePreScanSettings().getProbeRadius(),
      xmlSettings.getImagePreScanSettings().getScanLinePitch(), xmlSettings.getImagePreScanSettings().isTransducerConvex(), xmlSettings.getImagePreScanSettings().getAxialResolution()));
#else
    throw(vpException(vpException::fatalEtrror, "Requires xml2 library"));
#endif //VISP_HAVE_XML2
  }
  else if (headerFormat == FORMAT_MHD) {
    //header parsing
    usMetaHeaderParser mhdParser;
    mhdParser.read(headerFileName);
    if (mhdParser.getImageType() != usMetaHeaderParser::PRESCAN_2D && mhdParser.getImageType() != usMetaHeaderParser::NOT_SET) {
      throw(vpException(vpException::badValue, "Reading a non pre-scan 2D image!"));
    }
    if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
      throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
    }
    //resizing image in memory
    preScanImage.resize(mhdParser.getANumber(), mhdParser.getLineNumber());

    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

    usImagePreScanSettings settings;
    settings.setProbeRadius(mhdHeader.probeRadius);
    settings.setScanLinePitch(mhdHeader.scanLinePitch);
    settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
    settings.setAxialResolution(mhdParser.getAxialResolution());
    preScanImage.setImageSettings(settings);

    //data parsing
    usRawFileParser rawParser;
    rawParser.read(preScanImage, mhdParser.getRawFileName());
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Write 3D unsigned char pre-scan ultrasound image.
* @param preScanImage The pre-scan image to write.
* @param headerFileName The image file name to write.
*/
void usImageIo::write(const usImagePreScan3D<unsigned char> &preScanImage, const std::string headerFileName) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    write(preScanImage, headerFileName, ".png");
  }
  else if (headerFormat == FORMAT_MHD) {
    write(preScanImage, headerFileName, ".raw");
  }
}

/**
* Write 3D unsigned char pre-scan ultrasound image.
* @param preScanImage The pre-scan image to write.
* @param headerFileName The image file name to write.
* @param imageExtesion2D The 2D image extension.
*/
void usImageIo::write(const usImagePreScan3D<unsigned char> &preScanImage, const std::string headerFileName, const std::string imageExtesion2D) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtesion2D);
#ifdef VISP_HAVE_XML2
  //case of a set of successive 2D frames
#else
    throw(vpException(vpException::fatalError, "Requires xml2"));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    if (imageExtesion2D != ".raw") {
      throw(vpException(vpException::fatalError, "mhd files goes with .raw image extension"));
    }
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(".raw");
    //filling header
    usMetaHeaderParser::MHDHeader header;
    header.numberOfDimensions = 3;
    header.elementType = usMetaHeaderParser::MET_UCHAR;
    header.imageType = usMetaHeaderParser::PRESCAN_3D;
    header.elementSpacing[0] = 1;
    header.elementSpacing[1] = 1;
    header.elementSpacing[2] = 1;
    header.dim[0] = preScanImage.getDimX();
    header.dim[1] = preScanImage.getDimY();
    header.dim[2] = preScanImage.getDimZ();
    header.msb = false;
    header.MHDFileName = headerFileName;
    //remove full path for image file name (located in the same directory as the mhd
    header.rawFileName = vpIoTools::getName(imageFileName);
    header.isTransducerConvex = preScanImage.isTransducerConvex();
    header.probeRadius = preScanImage.getProbeRadius();
    header.scanLinePitch = preScanImage.getScanLinePitch();
    header.motorType = preScanImage.getMotorType();
    header.motorRadius = preScanImage.getMotorRadius();
    header.framePitch = preScanImage.getFramePitch();
    //writing in file
    usMetaHeaderParser mhdParser;
    mhdParser.setMHDHeader(header);
    mhdParser.setAxialResolution(preScanImage.getAxialResolution());
    mhdParser.parse();

    //filling raw
    usRawFileParser rawParser;
    rawParser.write(preScanImage, imageFileName);
  }
  else {
    throw(vpException(vpException::fatalError, "Unknown extension."));
  }
}

/**
* Read 3D unsigned char pre-scan ultrasound image.
* @param [out] preScanImage The pre-scan image to read.
* @param [in] headerFileName The image file name to read, with .xml extension.
*/
void usImageIo::read(usImagePreScan3D<unsigned char> &preScanImage,const std::string headerFileName) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    //case of a set of sucessive 2D frames
#else
    throw(vpException(vpException::fatalEtrror, "Requires xml2 library"));
#endif //VISP_HAVE_XML2
  }
  else if (headerFormat == FORMAT_MHD) {
    //header parsing
    usMetaHeaderParser mhdParser;
    mhdParser.read(headerFileName);
    if (mhdParser.getImageType() != usMetaHeaderParser::PRESCAN_3D && mhdParser.getImageType() != usMetaHeaderParser::NOT_SET) {
      throw(vpException(vpException::badValue, "Reading a non pre-scan 3D image!"));
    }
    if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
      throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
    }
    //resizing image in memory
    preScanImage.resize(mhdParser.getANumber(), mhdParser.getLineNumber(), mhdParser.getFrameNumber());

    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

    usImagePreScanSettings settings;
    settings.setProbeRadius(mhdHeader.probeRadius);
    settings.setScanLinePitch(mhdHeader.scanLinePitch);
    settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
    settings.setAxialResolution(mhdParser.getAxialResolution());
    preScanImage.setImageSettings(settings);

    usMotorSettings motorSettings;
    motorSettings.setMotorRadius(mhdHeader.motorRadius);
    motorSettings.setFramePitch(mhdHeader.framePitch);
    motorSettings.setMotorType(mhdHeader.motorType);
    preScanImage.setMotorSettings(motorSettings);

    //data parsing
    usRawFileParser rawParser;
    rawParser.read(preScanImage, mhdParser.getRawFileName());
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Write 2D double pre-scan ultrasound image.
* @param preScan2DImage The pre-scan image to write.
* @param headerFileName The image file name to write.
* @param imageExtesion2D The 2D image extension.
*/
void usImageIo::write(const usImagePreScan2D<double> &preScan2DImage,
                      const std::string headerFileName, const std::string imageExtesion2D)
{

}

/**
* Read 2D double pre-scan ultrasound image.
* @param [out] preScan2D The pre-scan image to read.
* @param [in] headerFileName The image file name to read.
*/
void usImageIo::read(usImagePreScan2D<double> &preScan2D,std::string headerFileName) {

}

/**
* Write 3D double pre-scan ultrasound image.
* @param preScan3DImage The pre-scan image to write.
* @param headerFileName The image file name to write.
* @param imageExtesion2D The 2D image extension.
*/
void usImageIo::write(const usImagePreScan3D<double> &preScan3DImage, const std::string headerFileName, const std::string imageExtesion2D) {

}

/**
* Read 3D double pre-scan ultrasound image.
* @param [out] preScan3DImage The pre-scan image to read.
* @param [in] headerFileName The image file name to read.
*/
void usImageIo::read(usImagePreScan3D<double> &preScan3DImage,std::string headerFileName) {

}

/**
* Write 2D post-scan ultrasound image and settings.
* @param postScanImage Image to write.
* @param headerFileName The header file name with the desired extension.
*/
void usImageIo::write(const usImagePostScan2D<unsigned char> &postScanImage, const std::string headerFileName) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    write(postScanImage, headerFileName, ".png");
  }
  else if(headerFormat == FORMAT_MHD){
    write(postScanImage, headerFileName, ".raw");
  }
}

/**
* Write 2D post-scan ultrasound image and settings.
* @param postScanImage Image to write.
* @param headerFileName The header file name with the desired extension.
* @param imageExtesion2D The 2D image extension.
*/
void usImageIo::write(const usImagePostScan2D<unsigned char> &postScanImage, const std::string headerFileName, const std::string imageExtesion2D) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(imageExtesion2D);
    try {
      //writing image
      vpImageIo::writePNG(postScanImage, imageFileName);
      //writing xml file using xml parser
      usImageSettingsXmlParser xmlSettings;
      xmlSettings.setImagePostScanSettings(postScanImage);
      xmlSettings.setImageType(usImageSettingsXmlParser::IMAGE_TYPE_POSTSCAN);
      //just writing the image file name without parents directories (header and image are in the same directory).
      imageFileName = vpIoTools::getName(imageFileName);
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
    if (imageExtesion2D != ".raw" && imageExtesion2D != ".RAW" ) {
      throw(vpException(vpException::fatalError, "mhd files goes with .raw image extension"));
    }
    //mhd writing
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(".raw");
    //filling header
    usMetaHeaderParser::MHDHeader header;
    header.numberOfDimensions = 2;
    header.elementType = usMetaHeaderParser::MET_UCHAR;
    header.imageType = usMetaHeaderParser::POSTSCAN_2D;
    header.elementSpacing[0] = 1;
    header.elementSpacing[1] = 1;
    header.dim[0] = postScanImage.getDimX();
    header.dim[1] = postScanImage.getDimY();
    header.msb = false;
    header.MHDFileName = headerFileName;
    //remove full path for image file name (located in the same directory as the mhd
    header.rawFileName = vpIoTools::getName(imageFileName);
    header.isTransducerConvex = postScanImage.isTransducerConvex();
    header.probeRadius = postScanImage.getProbeRadius();
    header.scanLinePitch = postScanImage.getScanLinePitch();
    //writing in file
    usMetaHeaderParser mhdParser;
    mhdParser.setMHDHeader(header);
    mhdParser.setHeightResolution(postScanImage.getHeightResolution());
    mhdParser.setWidthResolution(postScanImage.getWidthResolution());
    mhdParser.parse();

    //filling raw
    usRawFileParser rawParser;
    rawParser.write(postScanImage, imageFileName);
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
      std::cout << "file name : " << headerFileName << std::endl;
      throw e;
    }
    std::string fullImageFileName = vpIoTools::getParent(headerFileName) + vpIoTools::path("/") + xmlSettings.getImageFileName();
    vpImageIo::read(postScanImage, fullImageFileName);

    postScanImage.setImageSettings(usImagePostScanSettings(xmlSettings.getImagePostScanSettings().getProbeRadius(), 
      xmlSettings.getImagePostScanSettings().getScanLinePitch(),
      xmlSettings.getImagePostScanSettings().isTransducerConvex(),
      xmlSettings.getImagePostScanSettings().getHeightResolution(), xmlSettings.getImagePostScanSettings().getWidthResolution()));
#else
    throw(vpException(vpException::fatalError, "Requires xml2 library"));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    //mhd reading
    //header parsing
    usMetaHeaderParser mhdParser;
    mhdParser.read(headerFileName);
    if (mhdParser.getImageType() != usMetaHeaderParser::POSTSCAN_2D && mhdParser.getImageType() != usMetaHeaderParser::NOT_SET) {
      throw(vpException(vpException::badValue, "Reading a non post-scan 2D image!"));
    }
    if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
      throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
    }
    //resizing image in memory
    postScanImage.resize(mhdParser.getANumber(), mhdParser.getFrameNumber());

    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

    usImagePostScanSettings settings;
    settings.setProbeRadius(mhdHeader.probeRadius);
    settings.setScanLinePitch(mhdHeader.scanLinePitch);
    settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
    settings.setHeightResolution(mhdParser.getHeightResolution());
    settings.setWidthResolution(mhdParser.getWidthResolution());
    postScanImage.setImageSettings(settings);
    postScanImage.resize(mhdHeader.dim[0], mhdHeader.dim[1]);
    //data parsing
    usRawFileParser rawParser;
    rawParser.read(postScanImage, mhdParser.getRawFileName());
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Write 3D post-scan ultrasound image and settings
* @param postScanImage Image to write.
* @param headerFileName The header file name.
*/
void usImageIo::write(const usImagePostScan3D<unsigned char> &postScanImage, const std::string headerFileName) {
  //checking header type
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
    write(postScanImage, headerFileName, ".png");
  }
  else if (headerFormat == FORMAT_MHD) {
    write(postScanImage, headerFileName, ".raw");
  }
}

/**
* Write 3D post-scan ultrasound image and settings
* @param postScanImage Image to write.
* @param headerFileName The header file name.
* @param imageExtesion2D The 2D image extension.
*/
void usImageIo::write(const usImagePostScan3D<unsigned char> &postScanImage, const std::string headerFileName, const std::string imageExtesion2D)
{
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    //case of a set of sucessive 2D frames
#else
    throw(vpException(vpException::fatalError, "Requires xml2 library"));
#endif
  }
  else if (headerFormat == FORMAT_MHD) {
    if (imageExtesion2D != ".raw") {
      throw(vpException(vpException::fatalError, "mhd files goes with .raw image extension"));
    }
    std::string imageFileName = vpIoTools::splitChain(headerFileName, ".")[0].append(".raw");
    //filling header
    usMetaHeaderParser::MHDHeader header;
    header.numberOfDimensions = 3;
    header.elementType = usMetaHeaderParser::MET_UCHAR;
    header.imageType = usMetaHeaderParser::POSTSCAN_3D;
    header.elementSpacing[0] = postScanImage.getElementSpacingX();
    header.elementSpacing[1] = postScanImage.getElementSpacingY();
    header.elementSpacing[2] = postScanImage.getElementSpacingZ();
    header.dim[0] = postScanImage.getDimX();
    header.dim[1] = postScanImage.getDimY();
    header.dim[2] = postScanImage.getDimZ();
    header.msb = false;
    header.MHDFileName = headerFileName;
    //remove full path for image file name (located in the same directory as the mhd
    header.rawFileName = vpIoTools::getName(imageFileName);
    header.isTransducerConvex = postScanImage.isTransducerConvex();
    header.motorType = postScanImage.getMotorType();
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
    rawParser.write(postScanImage, imageFileName);
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}

/**
* Read 3D post-scan ultrasound image
* @param [out] postScanImage The post-scan image to read.
* @param [in] headerFileName The mhd file name with .mhd extenstion.
*/
void usImageIo::read(usImagePostScan3D<unsigned char> &postScanImage,std::string headerFileName) {
  usImageIo::usHeaderFormatType headerFormat = getHeaderFormat(headerFileName);
  if (headerFormat == FORMAT_XML) {
#ifdef VISP_HAVE_XML2
    //case of a set of successive 2D frames
#else
    throw(vpException(vpException::fatalError, "Requires xml2 library"));
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
    postScanImage.resize(mhdParser.getANumber(),mhdParser.getLineNumber(),mhdParser.getFrameNumber());

    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();

    usImagePostScanSettings settings;
    settings.setProbeRadius(mhdHeader.probeRadius);
    settings.setScanLinePitch(mhdHeader.scanLinePitch);
    settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
    settings.setWidthResolution(mhdParser.getWidthResolution());
    settings.setHeightResolution(mhdParser.getHeightResolution());
    postScanImage.setImageSettings(settings);
    usMotorSettings motorSettings;
    motorSettings.setMotorRadius(mhdHeader.motorRadius);
    motorSettings.setFramePitch(mhdHeader.framePitch);
    motorSettings.setMotorType(mhdHeader.motorType);
    postScanImage.setMotorSettings(motorSettings);
      

    //data parsing
    usRawFileParser rawParser;
    rawParser.read(postScanImage,mhdParser.getRawFileName());
  }
  else
    throw(vpException(vpException::fatalError, "Unknown header format."));
}
