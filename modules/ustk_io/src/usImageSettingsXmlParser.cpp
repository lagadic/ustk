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
 * @file usImageSettingsXmlParser.cpp
 * @brief Input/output operations between ultrasound image settings and the assiciated xml files.
 */
#include<visp3/ustk_io/usImageSettingsXmlParser.h>
#ifdef VISP_HAVE_XML2
usImageSettingsXmlParser::usImageSettingsXmlParser()
 : m_imageSettings(), m_imageFileName()
{
  nodeMap["settings"] = CODE_XML_SETTINGS;
  nodeMap["scanline_pitch"] = CODE_XML_SCANLINE_PITCH;
  nodeMap["probe_radius"] = CODE_XML_PROBE_RADIUS;
  nodeMap["is_convex"] = CODE_XML_IS_CONVEX;
  nodeMap["image_file_name"] = CODE_XML_ASSOCIATED_IMAGE_FILE_NAME;
}

usImageSettingsXmlParser::usImageSettingsXmlParser(usImageSettingsXmlParser& twinParser) : vpXmlParser(twinParser),
  m_imageSettings(twinParser.getImageSettings()), m_imageFileName(twinParser.getImageFileName())
{

}

usImageSettingsXmlParser& usImageSettingsXmlParser::operator =(const usImageSettingsXmlParser& twinparser)
{
  m_imageSettings = twinparser.getImageSettings();
  m_imageFileName = twinparser.getImageFileName();

  return *this;
}

usImageSettingsXmlParser::~usImageSettingsXmlParser()
{
  
}

void
usImageSettingsXmlParser::readMainClass (xmlDocPtr doc, xmlNodePtr node)
{
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case CODE_XML_SCANLINE_PITCH:
          this->m_imageSettings.setScanLinePitch(xmlReadFloatChild(doc, dataNode));
          break;
        case CODE_XML_PROBE_RADIUS:
          this->m_imageSettings.setProbeRadius(xmlReadFloatChild(doc, dataNode));
          break;
        case CODE_XML_IS_CONVEX:
          this->m_imageSettings.setImageConvex(xmlReadBoolChild(doc, dataNode));
          break;
        case CODE_XML_ASSOCIATED_IMAGE_FILE_NAME:{
          this->m_imageFileName = xmlReadStringChild(doc, dataNode);
        }break;
        default:
          vpTRACE("unknown tag in readConfigNode : %d, %s", iter_data->second, (iter_data->first).c_str());
          break;
        }
      }
    }
  }
}

void 
usImageSettingsXmlParser::writeMainClass(xmlNodePtr node)
{
  xmlWriteFloatChild(node, (const char*)"scanline_pitch", m_imageSettings.getScanLinePitch());
  xmlWriteFloatChild(node, (const char*)"probe_radius", m_imageSettings.getProbeRadius());
  xmlWriteBoolChild(node, (const char*)"is_convex", m_imageSettings.isImageConvex());
  xmlWriteCharChild(node, (const char*)"image_file_name", m_imageFileName.c_str());
}

void usImageSettingsXmlParser::setImageSettings(float probeRadius, float scanLinePitch, bool isImageConvex)
{
  m_imageSettings.setProbeRadius(probeRadius);
  m_imageSettings.setScanLinePitch(scanLinePitch);
  m_imageSettings.setImageConvex(isImageConvex);
}
#endif //VISP_HAVE_XML2

