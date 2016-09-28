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
* Write 2D rf ultrasound image
*/
void usImageIo::write(usImageRF2D &imageRf2D, const std::string filename) {

}

/**
* Read 2D rf ultrasound image
*/
void usImageIo::read(usImageRF2D &imageRf2D, const std::string filename) {

}

/**
* Write 3D rf ultrasound image
*/
void usImageIo::write(usImageRF3D &imageRf3D, const std::string filename) {

}

/**
* Read 3D rf ultrasound image
*/
void usImageIo::readRF3D(usImageRF3D &imageRf3,const std::string filename) {

}

#ifdef VISP_HAVE_XML2
/**
* Write 2D unsigned char prescan ultrasound image
*/
void usImageIo::writeXml(usImagePreScan2D<unsigned char> &preScanImage, const std::string imageFilename) {
    try {
        //writing image
        vpImageIo::write(preScanImage, imageFilename);
        //writing xml
        usImageSettingsXmlParser xmlSettings;
        xmlSettings.setImagePreScanSettings(preScanImage);
        xmlSettings.setImageFileName(imageFilename);
        //get xml filename from imageFilename
        std::vector<std::string> splittedFileName = vpIoTools::splitChain(imageFilename, ".");
        std::string xmlFileName = splittedFileName[0] + ".xml";
        xmlSettings.save(xmlFileName);
    }
    catch (std::exception e) {
        std::cout << "Error writing postScan image : " << std::endl;
        std::cout << e.what() << std::endl;
    }
}

/**
* Read 2D unsigned char prescan ultrasound image
*/
void usImageIo::readXml(usImagePreScan2D<unsigned char> &preScanImage,const std::string imageFilename)
{
    //get xml filename from imageFilename
    std::vector<std::string> splittedFileName = vpIoTools::splitChain(imageFilename, ".");
    std::string xmlFileName = splittedFileName[0] + ".xml";
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

    preScanImage.setImageSettings(xmlSettings.getImageSettings());
    preScanImage.setAxialResolution(xmlSettings.getAxialResolution());
}
#endif //VISP_HAVE_XML2

/**
* Write 3D unsigned char prescan ultrasound image
*/
void usImageIo::write(usImagePreScan3D<unsigned char> &preScanImage, const std::string filename) {

}

/**
* Read 3D unsigned char prescan ultrasound image
*/
void usImageIo::read(usImagePreScan3D<unsigned char> &preScanImage,const std::string filename) {

}

/**
* Write 2D double prescan ultrasound image
*/
void usImageIo::write(usImagePreScan2D<double> &preScan2DImage, const std::string filename) {

}

/**
* Read 2D double prescan ultrasound image
*/
void usImageIo::read(usImagePreScan2D<double> &preScan2D,std::string filename) {

}

/**
* Write 3D double prescan ultrasound image
*/
void usImageIo::write(usImagePreScan3D<double> &preScan3DImage, const std::string filename) {

}

/**
* Read 3D double prescan ultrasound image
*/
void usImageIo::readPreScan3DDouble(usImagePreScan3D<double> &preScan3DImage,std::string filename) {

}

#ifdef VISP_HAVE_XML2
/**
* Write 2D postscan ultrasound image and settings
* @param postScanImage Image to write
* @param filename The file name without extenstion (same name for png and xml);
*/
void usImageIo::writeXmlPng(usImagePostScan2D &postScanImage, const std::string filename) {
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
void usImageIo::readPostScan2DFromXml(usImagePostScan2D &postScanImage,const std::string xmlFilename) {
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
}
#endif //VISP_HAVE_XML2

/**
* Write 3D postscan ultrasound image and settings
*/
void usImageIo::write(usImagePostScan3D &postScanImage, const std::string filename) {

}

/**
* Read 3D postscan ultrasound image
*/
void usImageIo::readPostScan3D(usImagePostScan3D &postScanImage,std::string mhdFileName) {
    usMetaHeaderParser mhdParser;
    usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.readMHDHeader(mhdFileName.c_str());

    std::cout << "mhdFileName: " << mhdHeader.mhdFileName << std::endl;
    std::cout << "rawFileName: " << mhdHeader.rawFileName << std::endl;
    std::cout << "numberOfDimensions: " << mhdHeader.numberOfDimensions << std::endl;
    std::cout << "numberOfChannels: " << mhdHeader.numberOfChannels << std::endl;
    std::cout << "elementType: " << mhdHeader.elementType << std::endl;
    std::cout << "dim: " << mhdHeader.dim[0] << ", " << mhdHeader.dim[1] << ", " << mhdHeader.dim[2] << ", " << mhdHeader.dim[3] << std::endl;
    std::cout << "elementSpacing: " << mhdHeader.elementSpacing[0] << ", " << mhdHeader.elementSpacing[1] << ", " << mhdHeader.elementSpacing[2] << ", " << mhdHeader.elementSpacing[3] << std::endl;
    std::cout << "position: " << mhdHeader.position[0] << ", " << mhdHeader.position[1] << ", " << mhdHeader.position[2] << ", " << mhdHeader.position[3] << std::endl;
    std::cout << "headerSize: " << mhdHeader.headerSize << std::endl;
    std::cout << "msb: " << mhdHeader.msb << std::endl;
    std::cout << "imageType: " << mhdHeader.imageType << std::endl;
    return usImagePostScan3D();
}
