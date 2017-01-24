/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
* @file usMetaHeaderParser.h
* @brief Input/output operations between ultrasound image settings and the assiciated mhd files.
*/

#ifndef __usMetaHeaderParser_h_
#define __usMetaHeaderParser_h_

#include <string>
#include <map>
#include <cstdlib>
#include <string>
#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>

#include <visp3/core/vpConfig.h>
#include <visp3/ustk_core/us.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>

/**
 * @class usMetaHeaderParser
 * @brief Meta header data (MHD) parser.
 * @ingroup module_ustk_io
 */
class VISP_EXPORT usMetaHeaderParser {

public:

  typedef enum {
    MET_UNKNOWN = -1,
    MET_UCHAR,
    MET_SHORT,
    MET_DOUBLE,
  }ElementType;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct MHDHeader
  {
    std::string MHDFileName;
    std::string rawFileName;
    unsigned int  numberOfDimensions;
    int  numberOfChannels;
    ElementType elementType;
    int dim[4];
    double elementSpacing[4];
    double position[4];
    int headerSize;
    bool msb;
    us::ImageType imageType;
    bool isTransducerConvex;
    usMotorSettings::usMotorType motorType;
    double transducerRadius;
    double scanLinePitch;
    double motorRadius;
    double framePitch;
    unsigned int scanLineNumber; // for post-scan
    unsigned int frameNumber; // for post-scan
  };
#endif //DOXYGEN_SHOULD_SKIP_THIS

  //Constructor
  usMetaHeaderParser();
  usMetaHeaderParser(const std::string &MHDFilename);
  //Desctructor
  virtual ~usMetaHeaderParser();

  // Data accessors.
  double getAxialResolution() const { return m_axialResolution; }
  ElementType getElementType() const { return header.elementType; }
  double getHeightResolution() const { return m_heightResolution; }
  us::ImageType getImageType() const { return header.imageType; }
  MHDHeader getMHDHeader() const { return header; }
  usMotorSettings getMotorSettings() const {return m_motorSettings;}
  std::string getRawFileName() const {return header.rawFileName;}
  usTransducerSettings getTransducerSettings() const {return m_transducerSettings;}
  double getWidthResolution() const { return m_widthResolution; }

  //comparison
  bool operator ==(usMetaHeaderParser const& other);

  void parse();

  void read(const std::string &filename);
  void readMHDHeader(const std::string &fileName);

  //Setters
  void setAxialResolution(const double axialresolution);
  void setHeightResolution(const double heightResolution);
  void setImageFileName(const std::string &imageFileName);
  void setMHDHeader(MHDHeader header);
  void setMotorSettings(const usMotorSettings &motorSettings);
  void setRawFileName(const std::string &rawFileName);
  void setTransducerSettings(const usTransducerSettings &transducerSettings);
  void setWidthResolution(const double widthResolution);

private :
  usTransducerSettings m_transducerSettings;
  usMotorSettings m_motorSettings;
  double m_axialResolution;
  double m_heightResolution;
  double m_widthResolution;

  MHDHeader header;
  std::map<std::string, int> imageTypeMap;
  std::map<std::string, int> elementTypeMap;
  std::map<int ,std::string> imageTypeReverseMap;
  std::map<int ,std::string> elementTypeReverseMap;
};
#endif //US_META_HEADER_PARSER_H
