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
* @file usMetaHeaderParser.cpp
* @brief Input/output operations between ultrasound settings and mhd files.
*/
#include<visp3/core/vpException.h>
#include <visp3/ustk_io/usMetaHeaderParser.h>
#include <algorithm>
#include <string>

/**
* Default constructor.
*/
usMetaHeaderParser::usMetaHeaderParser()
{
  elementTypeMap["MET_UCHAR"] = MET_UCHAR;
  elementTypeMap["MET_SHORT"] = MET_SHORT;
  elementTypeMap["MET_DOUBLE"] = MET_DOUBLE;

  imageTypeMap["RF_2D"] = RF_2D;
  imageTypeMap["RF_3D"] = RF_3D;
  imageTypeMap["PRESCAN_2D"] = PRESCAN_2D;
  imageTypeMap["PRESCAN_3D"] = PRESCAN_3D;
  imageTypeMap["POSTSCAN_2D"] = POSTSCAN_2D;
  imageTypeMap["POSTSCAN_3D"] = POSTSCAN_3D;
}

/**
* Destructor.
*/
usMetaHeaderParser::~usMetaHeaderParser()
{

}

/**
* Reading method.
* @param fileName the mhd file to read.
*/
void  usMetaHeaderParser::readMHDHeader(const std::string &fileName)
{
  std::string pathPrefix;
  std::string tmp;

  //splitPathPrefixAndFileName(std::string(fileName), pathPrefix, tmp);

  std::string keyword, keyval;

  this->header.numberOfDimensions = 0;
  this->header.MHDFileName = fileName;
  this->header.rawFileName = "";
  this->header.numberOfChannels = 1;
  this->header.dim[0] = 1;
  this->header.dim[1] = 1;
  this->header.dim[2] = 1;
  this->header.dim[3] = 1;
  this->header.elementSpacing[0] = 1.0;
  this->header.elementSpacing[1] = 1.0;
  this->header.elementSpacing[2] = 1.0;
  this->header.elementSpacing[3] = 1.0;
  this->header.position[0] = 0.0;
  this->header.position[1] = 0.0;
  this->header.position[2] = 0.0;
  this->header.position[3] = 0.0;
  this->header.msb = false;
  this->header.headerSize = 0;
  this->header.imageType = NOT_SET;
  this->header.isTransducerConvex = false;
  this->header.motorType = usMotorSettings::LinearMotor;
  this->header.transducerRadius = 0.0;
  this->header.scanLinePitch = 0.0;
  this->header.motorRadius = 0.0;
  this->header.framePitch = 0.0;

  std::ifstream file;
  file.open(fileName.c_str(), std::ifstream::in);

  if (!file.good()) {
    throw vpException(vpException::fatalError, std::string("Error opening .mhd file."));
  }

  std::string::iterator it;
  while (file.good())
  {
    std::getline(file, keyword, '=');
    it=keyword.end();
    keyword.erase(std::remove(keyword.begin(),keyword.end(),' '),it);
    if (keyword == "NDims")
    {
      file >> this->header.numberOfDimensions;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "DimSize")
    {
      for (unsigned int i = 0; i < this->header.numberOfDimensions; i++)
        file >> this->header.dim[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ElementSpacing")
    {
      for (unsigned int i = 0; i < this->header.numberOfDimensions; i++)
        file >> this->header.elementSpacing[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "Position")
    {
      for (unsigned int i = 0; i < this->header.numberOfDimensions; i++)
        file >> this->header.position[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ImagePositionPatient")
    {
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ElementByteOrderMSB")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->header.msb = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "BinaryDataByteOrderMSB")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->header.msb = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "ElementNumberOfChannels")
    {
      file >> this->header.numberOfChannels;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ElementType")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(), keyval.end(), '\r'), it);
      std::map<std::string, int>::iterator mapIt = elementTypeMap.find(keyval);
      this->header.elementType = ((mapIt != elementTypeMap.end()) ? (ElementType)mapIt->second : MET_UNKNOWN);
    }
    else if (keyword == "HeaderSize")
    {
      file >> this->header.headerSize;
      std::getline(file, keyval, '\n');
      if ((this->header.headerSize) && (this->header.headerSize != -1)) {
        std::cout <<   "Warning: " << this->header.headerSize << " bytes this header" << std::endl;
      }
    }
    else if (keyword == "ElementDataFile")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->header.rawFileName = keyval;
    }
    else if (keyword == "UltrasoundImageType")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      std::map<std::string, int>::iterator mapIt = imageTypeMap.find(keyval);
      this->header.imageType = ((mapIt != imageTypeMap.end()) ? (ImageType)mapIt->second : UNKNOWN );
    }
    else if (keyword == "ScanLinePitch")
    {
      file >> this->header.scanLinePitch;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ScanLineNumber")
    {
      file >> this->header.scanLineNumber;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "TransducerRadius")
    {
      file >> this->header.transducerRadius;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "IsTransducerConvex")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->header.isTransducerConvex = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "FramePitch")
    {
      file >> this->header.framePitch;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "FrameNumber")
    {
      file >> this->header.frameNumber;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "MotorRadius")
    {
      file >> this->header.motorRadius;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "MotorType")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      if (keyval == "LinearMotor") {
        this->header.motorType = usMotorSettings::LinearMotor;
      }
      else if (keyval == "TiltingMotor") {
        this->header.motorType = usMotorSettings::TiltingMotor;
      }
      else if (keyval == "RotationalMotor") {
        this->header.motorType = usMotorSettings::RotationalMotor;
      }
      else {
        throw(vpException(vpException::badValue, "Unknown motor type"));
      }
    }
    else if (keyword == "AxialResolution")
    {
      if(this->header.imageType == POSTSCAN_2D || this->header.imageType == POSTSCAN_3D) {
        throw(vpException(vpException::badValue, "bad header file : trying to assign an axial resolution to a post-scan image"));
      }
      else {
        file >> this->m_axialResolution;
        std::getline(file, keyval, '\n');
      }
    }
    else if (keyword == "HeightResolution")
    {
      if(this->header.imageType == PRESCAN_2D || this->header.imageType == PRESCAN_3D || this->header.imageType == RF_2D || this->header.imageType == RF_3D) {
        throw(vpException(vpException::badValue, "bad header file : trying to assign a height resolution to a pre-scan image"));
      }
      else {
        file >> this->m_heightResolution;
        std::getline(file, keyval, '\n');
      }
    }
    else if (keyword == "WidthResolution")
    {
      if(this->header.imageType == PRESCAN_2D || this->header.imageType == PRESCAN_3D || this->header.imageType == RF_2D || this->header.imageType == RF_3D) {
        throw(vpException(vpException::badValue, "bad header file : trying to assign a width resolution to a pre-scan image"));
      }
      else {
        file >> this->m_widthResolution;
        std::getline(file, keyval, '\n');
      }
    }
    else
    {
      if (keyword != "")
      {
        std::getline(file, keyval, '\n');
      }
    }
  }

  file.close();
}

/**
* Writing method.
*/
void usMetaHeaderParser::parse()
{
  // write header
  std::ofstream MHDfile;
  try {
    MHDfile.open(header.MHDFileName.c_str());
    //scientific noatation for numbers
    MHDfile << std::scientific;

    MHDfile << "NDims = " << header.numberOfDimensions << "\n";;

    if (header.numberOfDimensions == 2) {
      char str[100];
      sprintf(str, "DimSize = %d %d \n", header.dim[0], header.dim[1]);
      MHDfile << str;
      char str2[100];
      sprintf(str2, "ElementSpacing = %f %f \n", header.elementSpacing[0], header.elementSpacing[1]);
      MHDfile << str2;
    }
    if (header.numberOfDimensions == 3) {
      char str[100];
      sprintf(str, "DimSize = %d %d %d \n", header.dim[0], header.dim[1], header.dim[2]);
      MHDfile << str;
      char str2[100];
      sprintf(str2, "ElementSpacing = %f %f %f \n", header.elementSpacing[0], header.elementSpacing[1], header.elementSpacing[2]);
      MHDfile << str2;
    }

    if (header.elementType == MET_SHORT)
      MHDfile << "ElementType = " << "MET_SHORT" << "\n";
    else if (header.elementType == MET_DOUBLE)
      MHDfile << "ElementType = " << "MET_DOUBLE" << "\n";
    else if (header.elementType == MET_UCHAR)
      MHDfile << "ElementType = " << "MET_UCHAR" << "\n";
    else
      MHDfile << "ElementType = " << "MET_UNKNOWN" << "\n";

    MHDfile << "ElementByteOrderMSB = " << header.msb << "\n";

    MHDfile << "ElementDataFile = " << header.rawFileName << "\n";

    if (header.imageType == RF_2D) {
      MHDfile << "Comment = Availables ultrasound image types are RF_2D, RF_3D, PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D and POSTSCAN_3D.\n";
      MHDfile << "UltrasoundImageType = " << "RF_2D" << "\n";
      MHDfile << "Comment = True if probe transducer is convex, false if linear. \n";
      MHDfile << "IsTransducerConvex = " << header.isTransducerConvex << "\n";
      MHDfile << "Comment = Radius between the scanlines intersection and the first pixel of each line acquired. 0 if linear probe.\n";
      MHDfile << "TransducerRadius = " << header.transducerRadius << "\n";
      MHDfile << "Comment = Distance between 2 scanlines.\n";
      MHDfile << "ScanLinePitch = " << header.scanLinePitch << "\n";
      MHDfile << "Comment = The axial resolution is the distance in meters between two successives A-samples in a scanline.\n";
      MHDfile << "AxialResolution = " << this->m_axialResolution << "\n";
    }
    else if (header.imageType == RF_3D) {
      MHDfile << "Comment = Availables ultrasound image types are RF_2D, RF_3D, PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D and POSTSCAN_3D.\n";
      MHDfile << "UltrasoundImageType = " << "RF_3D" << "\n";
      MHDfile << "Comment = True if probe transducer is convex, false if linear. \n";
      MHDfile << "IsTransducerConvex = " << header.isTransducerConvex << "\n";
      MHDfile << "Comment = Radius between the scanlines intersection and the first pixel of each line acquired. 0 if linear probe.\n";
      MHDfile << "TransducerRadius = " << header.transducerRadius << "\n";
      MHDfile << "Comment = Distance between 2 scanlines.\n";
      MHDfile << "ScanLinePitch = " << header.scanLinePitch << "\n";
      MHDfile << "Comment = Probe motor type : LinearMotor, TiltingMotor (for small roatations), or RotationalMotor (for 360 deg rotation).\n";
      if (header.motorType == usMotorSettings::LinearMotor) {
        MHDfile << "MotorType = " << "LinearMotor" << "\n";
      }
      else if (header.motorType == usMotorSettings::TiltingMotor) {
        MHDfile << "MotorType = " << "TiltingMotor" << "\n";
      }
      else if (header.motorType == usMotorSettings::RotationalMotor) {
        MHDfile << "MotorType = " << "RotationalMotor" << "\n";
      }
      MHDfile << "Comment = Only in 3d. Radius between the probe motor center and the first pixel of each line acquired. 0 if linear motor.\n";
      MHDfile << "MotorRadius = " << header.motorRadius << "\n";
      MHDfile << "Comment = Only in 3d. Distance between 2 successive frames.\n";
      MHDfile << "FramePitch = " << header.framePitch << "\n";
      MHDfile << "Comment = The axial resolution is the distance in meters between two successives A-samples in a scanline.\n";
      MHDfile << "AxialResolution = " << this->m_axialResolution << "\n";
    }
    else if (header.imageType == PRESCAN_2D) {
      MHDfile << "Comment = Availables ultrasound image types are RF_2D, RF_3D, PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D and POSTSCAN_3D.\n";
      MHDfile << "UltrasoundImageType = " << "PRESCAN_2D" << "\n";
      MHDfile << "Comment = True if probe transducer is convex, false if linear. \n";
      MHDfile << "IsTransducerConvex = " << header.isTransducerConvex << "\n";
      MHDfile << "Comment = Radius between the scanlines intersection and the first pixel of each line acquired. 0 if linear probe.\n";
      MHDfile << "TransducerRadius = " << header.transducerRadius << "\n";
      MHDfile << "Comment = Distance between 2 scanlines.\n";
      MHDfile << "ScanLinePitch = " << header.scanLinePitch << "\n";
      MHDfile << "Comment = The axial resolution is the distance in meters between two successives A-samples in a scanline.\n";
      MHDfile << "AxialResolution = " << this->m_axialResolution << "\n";
    }
    else if (header.imageType == PRESCAN_3D) {
      MHDfile << "Comment = Availables ultrasound image types are RF_2D, RF_3D, PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D and POSTSCAN_3D.\n";
      MHDfile << "UltrasoundImageType = " << "PRESCAN_3D" << "\n";
      MHDfile << "Comment = True if probe transducer is convex, false if linear. \n";
      MHDfile << "IsTransducerConvex = " << header.isTransducerConvex << "\n";
      MHDfile << "Comment = Radius between the scanlines intersection and the first pixel of each line acquired. 0 if linear probe.\n";
      MHDfile << "TransducerRadius = " << header.transducerRadius << "\n";
      MHDfile << "Comment = Distance between 2 scanlines.\n";
      MHDfile << "ScanLinePitch = " << header.scanLinePitch << "\n";
      MHDfile << "Comment = Probe motor type : LinearMotor, TiltingMotor (for small roatations), or RotationalMotor (for 360 deg rotation).\n";
      if (header.motorType == usMotorSettings::LinearMotor) {
        MHDfile << "MotorType = " << "LinearMotor" << "\n";
      }
      else if (header.motorType == usMotorSettings::TiltingMotor) {
        MHDfile << "MotorType = " << "TiltingMotor" << "\n";
      }
      else if (header.motorType == usMotorSettings::RotationalMotor) {
        MHDfile << "MotorType = " << "RotationalMotor" << "\n";
      }
      MHDfile << "Comment = Only in 3d. Radius between the probe motor center and the first pixel of each line acquired. 0 if linear motor.\n";
      MHDfile << "MotorRadius = " << header.motorRadius << "\n";
      MHDfile << "Comment = Only in 3d. Distance between 2 successive frames.\n";
      MHDfile << "FramePitch = " << header.framePitch << "\n";
      MHDfile << "Comment = The axial resolution is the distance in meters between two successives A-samples in a scanline.\n";
      MHDfile << "AxialResolution = " << this->m_axialResolution << "\n";
    }
    else if (header.imageType == POSTSCAN_2D) {
      MHDfile << "Comment = Availables ultrasound image types are RF_2D, RF_3D, PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D and POSTSCAN_3D.\n";
      MHDfile << "UltrasoundImageType = " << "POSTSCAN_2D" << "\n";
      MHDfile << "Comment = True if probe transducer is convex, false if linear. \n";
      MHDfile << "IsTransducerConvex = " << header.isTransducerConvex << "\n";
      MHDfile << "Comment = Radius between the scanlines intersection and the first pixel of each line acquired. 0 if linear probe.\n";
      MHDfile << "TransducerRadius = " << header.transducerRadius << "\n";
      MHDfile << "Comment = Distance between 2 scanlines.\n";
      MHDfile << "ScanLinePitch = " << header.scanLinePitch << "\n";
      MHDfile << "HeightResolution = " << this->m_heightResolution << "\n";
      MHDfile << "WidthResolution = " << this->m_widthResolution << "\n";
      MHDfile << "ScanLineNumber = " << header.scanLineNumber << "\n";
    }
    else if (header.imageType == POSTSCAN_3D) {
      MHDfile << "Comment = Availables ultrasound image types are RF_2D, RF_3D, PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D and POSTSCAN_3D.\n";
      MHDfile << "UltrasoundImageType = " << "POSTSCAN_3D" << "\n";
      MHDfile << "Comment = True if probe transducer is convex, false if linear. \n";
      MHDfile << "IsTransducerConvex = " << header.isTransducerConvex << "\n";
      MHDfile << "Comment = Radius between the scanlines intersection and the first pixel of each line acquired. 0 if linear probe.\n";
      MHDfile << "TransducerRadius = " << header.transducerRadius << "\n";
      MHDfile << "Comment = Distance between 2 scanlines.\n";
      MHDfile << "ScanLinePitch = " << header.scanLinePitch << "\n";
      MHDfile << "Comment = Probe motor type : LinearMotor, TiltingMotor (for small roatations), or RotationalMotor (for 360 deg rotation).\n";
      if (header.motorType == usMotorSettings::LinearMotor) {
        MHDfile << "MotorType = " << "LinearMotor" << "\n";
      }
      else if (header.motorType == usMotorSettings::TiltingMotor) {
        MHDfile << "MotorType = " << "TiltingMotor" << "\n";
      }
      else if (header.motorType == usMotorSettings::RotationalMotor) {
        MHDfile << "MotorType = " << "RotationalMotor" << "\n";
      }
      MHDfile << "Comment = Only in 3d. Radius between the probe motor center and the first pixel of each line acquired. 0 if linear motor.\n";
      MHDfile << "MotorRadius = " << header.motorRadius << "\n";
      MHDfile << "Comment = Only in 3d. Distance between 2 successive frames.\n";
      MHDfile << "FramePitch = " << header.framePitch << "\n";
      MHDfile << "HeightResolution = " << this->m_heightResolution << "\n";
      MHDfile << "WidthResolution = " << this->m_widthResolution << "\n";
      MHDfile << "ScanLineNumber = " << header.scanLineNumber << "\n";
      MHDfile << "FrameNumber = " << header.frameNumber << "\n";
    }
    else
      MHDfile << "UltrasoundImageType = " << "MET_UNKNOWN" << "\n";

    MHDfile.close();
  }
  catch (std::exception e) {
    std::cout << "Error opening file : " << e.what() << std::endl;
  }
}

/**
* Reading method.
* @param filename the mhd file to read.
*/
void usMetaHeaderParser::read(const std::string& filename)
{
  readMHDHeader(filename);

  //basic transducer settings
  this->m_transducerSettings.setTransducerRadius(header.transducerRadius);
  this->m_transducerSettings.setScanLinePitch(header.scanLinePitch);
  this->m_transducerSettings.setTransducerConvexity(header.isTransducerConvex);

  if(this->header.imageType == RF_3D || this->header.imageType == PRESCAN_3D || this->header.imageType == POSTSCAN_3D) {
    this->m_motorSettings.setMotorRadius(header.motorRadius);
    this->m_motorSettings.setFramePitch(header.framePitch);
    this->m_motorSettings.setMotorType(header.motorType);
  }
}

/**
* Transducer settings setter.
* @param transducerSettings usTransducerSettings to set.
*/
void usMetaHeaderParser::setTransducerSettings(const usTransducerSettings &transducerSettings)
{
  m_transducerSettings = transducerSettings;
}

/**
* Motor settings setter (for 3D images).
* @param motorSettings usMotorSettings to set.
*/
void usMetaHeaderParser::setMotorSettings(const usMotorSettings &motorSettings)
{
  m_motorSettings = motorSettings;
}

/**
* Image file name setter.
* @param imageFileName Image file name (with .raw extesion).
*/
void usMetaHeaderParser::setRawFileName(const std::string &imageFileName)
{
  header.rawFileName = imageFileName;
}

/**
* Image axial resolution setter (for rf or pre-scan images).
* @param axialresolution Image axial resolution.
*/
void usMetaHeaderParser::setAxialResolution(const double axialresolution)
{
  m_axialResolution = axialresolution;
}

/**
* Image height resolution setter (for post-scan images).
* @param heightResolution Image height resolution.
*/
void usMetaHeaderParser::setHeightResolution(const double heightResolution)
{
  m_heightResolution = heightResolution;
}

/**
* Image width resolution setter (for post-scan images).
* @param widthResolution Image width resolution.
*/
void usMetaHeaderParser::setWidthResolution(const double widthResolution)
{
  m_widthResolution = widthResolution;
}

/**
* Mhd header setter (to set all information contained in the mhd file).
* @param header MHDHeader to set.
*/
void usMetaHeaderParser::setMHDHeader(const MHDHeader header)
{
  this->header = header;
}
