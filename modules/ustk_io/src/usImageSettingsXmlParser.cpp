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
  : m_postScanSettings(usImagePostScanSettings()), m_preScanSettings(usImagePreScanSettings()), m_imageFileName(std::string("")), m_is_prescan(false)
{
  nodeMap["settings"] = CODE_XML_SETTINGS;
  nodeMap["image_type"] = CODE_XML_IMAGE_TYPE;
  nodeMap["scanline_pitch"] = CODE_XML_SCANLINE_PITCH;
  nodeMap["probe_radius"] = CODE_XML_PROBE_RADIUS;
  nodeMap["is_probe_convex"] = CODE_XML_IS_PROBE_CONVEX;
  nodeMap["axial_resolution"] = CODE_XML_AXIAL_RESOLUTION;
  nodeMap["height_resolution"] = CODE_XML_HEIGHT_RESOLUTION;
  nodeMap["width_resolution"] = CODE_XML_WIDTH_RESOLUTION;
  nodeMap["image_file_name"] = CODE_XML_ASSOCIATED_IMAGE_FILE_NAME;
}

usImageSettingsXmlParser::usImageSettingsXmlParser(usImageSettingsXmlParser& twinParser) : vpXmlParser(twinParser),
m_postScanSettings(twinParser.getImagePostScanSettings()), m_preScanSettings(twinParser.getImagePreScanSettings()), m_imageFileName(twinParser.getImageFileName()), m_is_prescan(false)
{

}

usImageSettingsXmlParser& usImageSettingsXmlParser::operator =(const usImageSettingsXmlParser& twinparser)
{
  m_postScanSettings = twinparser.getImagePostScanSettings();
  m_preScanSettings = twinparser.getImagePreScanSettings();
  m_imageFileName = twinparser.getImageFileName();

  return *this;
}

usImageSettingsXmlParser::~usImageSettingsXmlParser()
{
  
}

bool usImageSettingsXmlParser::operator ==(usImageSettingsXmlParser const& other)
{
  return (this->getImagePostScanSettings() == other.getImagePostScanSettings() &&
    this->getImagePreScanSettings() == other.getImagePreScanSettings() &&
    this->getImageFileName() == other.getImageFileName());
}

void
usImageSettingsXmlParser::readMainClass (xmlDocPtr doc, xmlNodePtr node)
{
  std::string value;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second) {
        case CODE_XML_IMAGE_TYPE:
          value = xmlReadStringChild(doc, dataNode);
          if(strcmp(value.c_str(), "postscan") != 0 &&
             strcmp(value.c_str(), "prescan") != 0 &&
             strcmp(value.c_str(), "rf") != 0) {
            throw(vpException(vpException::fatalError, std::string("unknown image type in xml file")));
            break;
          }
          this->m_is_prescan = (strcmp(value.c_str(), "postscan") ? true : false);
          break;
        case CODE_XML_AXIAL_RESOLUTION:
          if (this->m_is_prescan) {
            this->m_preScanSettings.setAxialResolution(xmlReadDoubleChild(doc, dataNode));
            break;
          }
          throw(vpException(vpException::fatalError, std::string("Trying to assign an axial resolution to a post-scan image !")));
          break;
        case CODE_XML_HEIGHT_RESOLUTION:
          if (this->m_is_prescan) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an height resolution to a pre-scan image !")));
            break;
          }
          this->m_postScanSettings.setHeightResolution(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_WIDTH_RESOLUTION :
          if (this->m_is_prescan) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an width resolution to a pre-scan image !")));
            break;
          }
          this->m_postScanSettings.setHeightResolution(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_SCANLINE_PITCH:
          this->m_postScanSettings.setScanLinePitch(xmlReadDoubleChild(doc, dataNode));
          this->m_preScanSettings.setScanLinePitch(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_PROBE_RADIUS:
          this->m_postScanSettings.setProbeRadius(xmlReadDoubleChild(doc, dataNode));
          this->m_preScanSettings.setProbeRadius(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_IS_PROBE_CONVEX:
          this->m_postScanSettings.setTransducerConvexity(xmlReadBoolChild(doc, dataNode));
          this->m_preScanSettings.setTransducerConvexity(xmlReadBoolChild(doc, dataNode));
          break;
        case CODE_XML_ASSOCIATED_IMAGE_FILE_NAME:
          this->m_imageFileName = xmlReadStringChild(doc, dataNode);
          break;
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
  if (m_is_prescan) {
    xmlWriteStringChild(node, "image_type", std::string("prescan"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_preScanSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_preScanSettings.getProbeRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_preScanSettings.isTransducerConvex());
    xmlWriteDoubleChild(node, "axial_resolution", m_preScanSettings.getAxialResolution());
    xmlWriteCharChild(node, "image_file_name", m_imageFileName.c_str());
  }
  else {
    xmlWriteStringChild(node, "image_type", std::string("postscan"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_postScanSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_postScanSettings.getProbeRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_postScanSettings.isTransducerConvex());
    xmlWriteDoubleChild(node, "width_resolution", m_postScanSettings.getWidthResolution());
    xmlWriteDoubleChild(node, "height_resolution", m_postScanSettings.getHeightResolution());
  }
  xmlWriteCharChild(node, "image_file_name", m_imageFileName.c_str());
}

void usImageSettingsXmlParser::setImagePreScanSettings(const usImagePreScanSettings imagePrescanSettings)
{
  m_preScanSettings = imagePrescanSettings;
}

void usImageSettingsXmlParser::setImagePostScanSettings(const usImagePostScanSettings& imagePostScanSettings)
{
  m_postScanSettings = imagePostScanSettings;
}

void usImageSettingsXmlParser::setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double axialResolution)
{
  m_preScanSettings.setTransducerConvexity(isTransducerConvex);
  m_preScanSettings.setProbeRadius(probeRadius);
  m_preScanSettings.setScanLinePitch(scanLinePitch);
  m_preScanSettings.setAxialResolution(axialResolution);
  m_is_prescan = true;
}

void usImageSettingsXmlParser::setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double widthResolution, double heightResolution)
{
  m_postScanSettings.setTransducerConvexity(isTransducerConvex);
  m_postScanSettings.setProbeRadius(probeRadius);
  m_postScanSettings.setScanLinePitch(scanLinePitch);
  m_postScanSettings.setHeightResolution(widthResolution);
  m_postScanSettings.setWidthResolution(heightResolution);
  m_is_prescan = false;
}

void usImageSettingsXmlParser::setImageFileName(std::string imageFileName)
{ 
  m_imageFileName = imageFileName;
}
#endif //VISP_HAVE_XML2
