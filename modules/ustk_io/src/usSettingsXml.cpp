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
#include<visp3/ustk_io/usSettingsXml.h>

usSettingsXml::usSettingsXml()
 : m_imageSettings(NULL), m_imageFileName("")
{
  nodeMap["settings"] = settings;
  nodeMap["scanline_pitch"] = scanline_pitch;
  nodeMap["probe_radius"] = probe_radius;
  nodeMap["is_convex"] = is_convex;
  nodeMap["image_file_name"] = image_file_name;
}
usSettingsXml::~usSettingsXml()
{
  
}
void
usSettingsXml::readMainClass (xmlDocPtr doc, xmlNodePtr node)
{
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case scanline_pitch:
          this->m_imageSettings->setScanLinePitch(xmlReadFloatChild(doc, dataNode));
          break;
        case probe_radius:
          this->m_imageSettings->setProbeRadius(xmlReadFloatChild(doc, dataNode));
          break;
        case is_convex:
          this->m_imageSettings->setImageConvex(xmlReadBoolChild(doc, dataNode));
          break;
        case image_file_name:{
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
usSettingsXml::writeMainClass(xmlNodePtr node)
{
  xmlWriteFloatChild(node, (const char*)"scanline_pitch", m_imageSettings->getScanLinePitch());
  xmlWriteFloatChild(node, (const char*)"probe_radius", m_imageSettings->getProbeRadius());
  xmlWriteBoolChild(node, (const char*)"is_convex", m_imageSettings->isImageConvex());
  xmlWriteCharChild(node, (const char*)"image_file_name", m_imageFileName.c_str());
}

