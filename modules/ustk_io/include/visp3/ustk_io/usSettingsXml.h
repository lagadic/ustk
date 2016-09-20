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
 * @file usSettingsXml.h
 * @brief Input/output operations between ultrasound image settings and the assiciated xml files.
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#ifndef US_SETTINGS_XML_H
#define US_SETTINGS_XML_H
#include <iostream>
#include <visp3/ustk_core/usImageSettings.h>
#include <visp3/core/vpXmlParser.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <string>

/**
 * @class usSettingsXml
 * @brief Input/output operations between ultrasound image settings and the assiciated xml files.
 */


class VISP_EXPORT usSettingsXml: public vpXmlParser
{
protected:
  usImageSettings* m_imageSettings;
  std::string m_imageFileName;
  
  typedef enum{
    settings,
    scanline_pitch,
    probe_radius,
    is_convex,
    image_file_name
  }dataToParse;
  
public:
  usSettingsXml();
  virtual ~usSettingsXml();
  
  // Data accessors.
  usImageSettings* getImageSettings() const {return m_imageSettings;}
  std::string getImageFileName() const {return m_imageFileName;}
  
  void setImageSettings(usImageSettings* imageSettings) { m_imageSettings = imageSettings;}
  void setImageFileName(std::string imageFileName) { m_imageFileName = imageFileName;}
  
protected:  
  virtual void readMainClass (xmlDocPtr doc, xmlNodePtr node);
  virtual void writeMainClass (xmlNodePtr node);
};
#endif //US_SETTINGS_XML_H
#endif //VISP_HAVE_XML2

