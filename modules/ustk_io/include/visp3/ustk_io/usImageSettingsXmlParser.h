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
#include <visp3/ustk_core/usImageSettings.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
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
    CODE_XML_IS_CONVEX,
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

  usImageSettingsXmlParser();
  usImageSettingsXmlParser(usImageSettingsXmlParser& twinParser);
  usImageSettingsXmlParser& operator =(const usImageSettingsXmlParser& twinparser);
  virtual ~usImageSettingsXmlParser();
  
  // Data accessors.
  usImageSettings getImageSettings() const {return m_imageSettings;}
  std::string getImageFileName() const {return m_imageFileName;}
  float getAxialResolution() const { return m_axialResolution; }
  float getHeightResolution() const { return m_heightResolution; }
  float getWidthResolution() const { return m_widthResolution; }
  
  //Data setters
  void setImagePreScanSettings(usImagePreScan2D<unsigned char> imagePrescan2D);
  void setImagePostScanSettings(usImagePostScan2D imagePostcan2D);
  void setImageSettings(float probeRadius, float scanLinePitch, bool isImageConvex, float axialResolution);
  void setImageSettings(float probeRadius, float scanLinePitch, bool isImageConvex, float widthResolution, float heightResolution);
  void setImageFileName(std::string imageFileName);

private:
  usImageSettings m_imageSettings;
  std::string m_imageFileName;

  //to manage different resolution types
  bool m_is_prescan;
  float m_axialResolution;
  float m_heightResolution;
  float m_widthResolution;

  virtual void readMainClass (xmlDocPtr doc, xmlNodePtr node);
  virtual void writeMainClass (xmlNodePtr node);
};
#endif //US_IMAGE_SETTINGS_XML_PARSER_H
#endif //VISP_HAVE_XML2

