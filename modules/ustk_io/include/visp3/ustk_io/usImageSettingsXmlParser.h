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
 * @file usImageSettingsXmlParser.h
 * @brief Input/output operations between ultrasound image settings and the assiciated xml files.
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#ifndef US_IMAGE_SETTINGS_XML_PARSER_H
#define US_IMAGE_SETTINGS_XML_PARSER_H
#include <iostream>
#include <visp3/ustk_core/usTransducerSettings.h>
#include <visp3/ustk_core/usImagePostScanSettings.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usMotorSettings.h>
#include <visp3/core/vpXmlParser.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <string>

/**
 * @class usImageSettingsXmlParser
 * @brief Input/output operations between ultrasound image settings and the assiciated xml files.
 */


class VISP_EXPORT usImageSettingsXmlParser: public vpXmlParser
{
public:
  typedef enum{
    CODE_XML_BAD = -1,
    CODE_XML_SETTINGS,
    CODE_XML_IMAGE_TYPE,
    CODE_XML_SCANLINE_PITCH,
    CODE_XML_PROBE_RADIUS,
    CODE_XML_IS_PROBE_CONVEX,
    CODE_XML_FRAME_PITCH,
    CODE_XML_MOTOR_RADIUS,
    CODE_XML_IS_MOTOR_ROTATING,
    CODE_XML_AXIAL_RESOLUTION,
    CODE_XML_HEIGHT_RESOLUTION,
    CODE_XML_WIDTH_RESOLUTION,
    CODE_XML_ASSOCIATED_IMAGE_FILE_NAME
  } vpXmlCodeType;

  typedef enum
  {
    SEQUENCE_OK,
    SEQUENCE_ERROR
  } vpXmlCodeSequenceType;

  typedef enum
  {
    IMAGE_TYPE_UNKNOWN = -1,
    IMAGE_TYPE_RF,
    IMAGE_TYPE_PRESCAN,
    IMAGE_TYPE_POSTSCAN
  } usImageType;
  
  usImageSettingsXmlParser();
  usImageSettingsXmlParser(usImageSettingsXmlParser& twinParser);
  usImageSettingsXmlParser& operator =(const usImageSettingsXmlParser& twinparser);
  virtual ~usImageSettingsXmlParser();
  
  //comparison
  bool operator ==(usImageSettingsXmlParser const& other);

  // Data accessors.
  std::string getImageFileName() const {return m_imageFileName;}
  usImagePostScanSettings getImagePostScanSettings() const {return m_postScanSettings;}
  usImagePreScanSettings getImagePreScanSettings() const { return m_preScanSettings; }
  usMotorSettings getMotorSettings() const { return m_motorSettings; }
  usImageType getImageType() const { return m_image_type; }
  bool isImage3D() const { return m_is_3D; }
  
  //Data setters
  void setImagePreScanSettings(const usImagePreScanSettings imagePrescanSettings);
  void setImagePostScanSettings(const usImagePostScanSettings& imagePostScanSettings);
  void usImageSettingsXmlParser::setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double axialResolution, usImageType image_type);
  void setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double widthResolution, double heightResolution);
  void setMotorSettings(const usMotorSettings &motorSettings);
  void setImageFileName(std::string imageFileName);
  void setImageType(usImageType image_type) { m_image_type = image_type; }

private:
  usImagePostScanSettings m_postScanSettings;
  usImagePreScanSettings m_preScanSettings;
  usMotorSettings m_motorSettings;
  std::string m_imageFileName;

  //to manage different resolution types
  usImageType m_image_type;
  bool m_is_3D;

  virtual void readMainClass (xmlDocPtr doc, xmlNodePtr node);
  virtual void writeMainClass (xmlNodePtr node);
};
#endif //US_IMAGE_SETTINGS_XML_PARSER_H
#endif //VISP_HAVE_XML2

