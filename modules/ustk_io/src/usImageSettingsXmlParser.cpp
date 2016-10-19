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

 /**
 * Default constructor.
 */
usImageSettingsXmlParser::usImageSettingsXmlParser()
  : m_postScanSettings(usImagePostScanSettings()), m_preScanSettings(usImagePreScanSettings()),
    m_motorSettings(usMotorSettings()), m_imageFileName(std::string("")),
    m_image_type(usImageSettingsXmlParser::IMAGE_TYPE_UNKNOWN), m_is_3D(false)
{
  nodeMap["settings"] = CODE_XML_SETTINGS;
  nodeMap["image_type"] = CODE_XML_IMAGE_TYPE;
  nodeMap["scanline_pitch"] = CODE_XML_SCANLINE_PITCH;
  nodeMap["probe_radius"] = CODE_XML_PROBE_RADIUS;
  nodeMap["is_probe_convex"] = CODE_XML_IS_PROBE_CONVEX;
  nodeMap["frame_pitch"] = CODE_XML_FRAME_PITCH;
  nodeMap["motor_radius"] = CODE_XML_MOTOR_RADIUS;
  nodeMap["motor_type"] = CODE_XML_MOTOR_TYPE;
  nodeMap["axial_resolution"] = CODE_XML_AXIAL_RESOLUTION;
  nodeMap["height_resolution"] = CODE_XML_HEIGHT_RESOLUTION;
  nodeMap["width_resolution"] = CODE_XML_WIDTH_RESOLUTION;
  nodeMap["image_file_name"] = CODE_XML_ASSOCIATED_IMAGE_FILE_NAME;
  //nodeMap["sequence_name"] = CODE_XML_SEQUENCE_NAME;
  nodeMap["sequence_frame_rate"] = CODE_XML_SEQUENCE_FRAME_RATE;
  nodeMap["sequence_start_number"] = CODE_XML_SEQUENCE_FIRST_IMAGE_NUMBER;
  nodeMap["sequence_stop_number"] = CODE_XML_SEQUENCE_LAST_IMAGE_NUMBER;
}

/**
* Copy constructor.
* @param twinParser usImageSettingsXmlParser to copy.
*/
usImageSettingsXmlParser::usImageSettingsXmlParser(usImageSettingsXmlParser& twinParser)
  : vpXmlParser(twinParser), m_postScanSettings(twinParser.getImagePostScanSettings()),
    m_preScanSettings(twinParser.getImagePreScanSettings()), m_motorSettings(),
    m_imageFileName(twinParser.getImageFileName()),
    m_image_type(twinParser.getImageType()), m_is_3D(twinParser.isImage3D())
{

}

/**
* Assignement operator.
* @param twinparser usImageSettingsXmlParser to assign.
*/
usImageSettingsXmlParser& usImageSettingsXmlParser::operator =(const usImageSettingsXmlParser& twinparser)
{
  m_postScanSettings = twinparser.getImagePostScanSettings();
  m_preScanSettings = twinparser.getImagePreScanSettings();
  m_imageFileName = twinparser.getImageFileName();
  m_motorSettings = twinparser.getMotorSettings();
  m_image_type = twinparser.getImageType();
  m_is_3D = twinparser.isImage3D();

  return *this;
}

/**
* Destructor.
*/
usImageSettingsXmlParser::~usImageSettingsXmlParser()
{
  
}

/**
* Comparaison operator.
* @param other usImageSettingsXmlParser to compare with.
*/
bool usImageSettingsXmlParser::operator ==(usImageSettingsXmlParser const& other)
{
  return (this->getImagePostScanSettings() == other.getImagePostScanSettings() &&
    this->getImagePreScanSettings() == other.getImagePreScanSettings() &&
    this->getImageFileName() == other.getImageFileName() &&
    this->getMotorSettings() == other.getMotorSettings() &&
    this->isImage3D() == other.isImage3D() &&
    this->getImageType() == other.getImageType());
}

/**
* Reading method, called by vpXmlParser::parse().
* @param doc a pointer representing the document
* @param node : the root node of the document
*/
void
usImageSettingsXmlParser::readMainClass (xmlDocPtr doc, xmlNodePtr node)
{
  std::string value;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case CODE_XML_IMAGE_TYPE:
          value = xmlReadStringChild(doc, dataNode);
          if (strcmp(value.c_str(), "postscan") == 0) {
            this->m_image_type = usImageSettingsXmlParser::IMAGE_TYPE_POSTSCAN;
          }
          else if (strcmp(value.c_str(), "prescan") == 0) {
            this->m_image_type = usImageSettingsXmlParser::IMAGE_TYPE_PRESCAN;
          }
          else if (strcmp(value.c_str(), "rf") == 0) {
            this->m_image_type = usImageSettingsXmlParser::IMAGE_TYPE_RF;
          }
          else
            throw(vpException(vpException::fatalError, std::string("unknown image type in xml file")));
          break;
          value = "";
        case CODE_XML_AXIAL_RESOLUTION:
          if (this->m_image_type == IMAGE_TYPE_PRESCAN || this->m_image_type == IMAGE_TYPE_RF) {
            this->m_preScanSettings.setAxialResolution(xmlReadDoubleChild(doc, dataNode));
          }
          else 
            throw(vpException(vpException::fatalError, std::string("Trying to assign an axial resolution to a post-scan image !")));
          break;
        case CODE_XML_HEIGHT_RESOLUTION:
          if (this->m_image_type == IMAGE_TYPE_RF || this->m_image_type == IMAGE_TYPE_PRESCAN) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an height resolution to a pre-scan image !")));
          }
          else 
            this->m_postScanSettings.setHeightResolution(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_WIDTH_RESOLUTION :
          if (this->m_image_type == IMAGE_TYPE_RF || this->m_image_type == IMAGE_TYPE_PRESCAN) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an width resolution to a pre-scan image !")));
          } else
            this->m_postScanSettings.setWidthResolution(xmlReadDoubleChild(doc, dataNode));
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
        case CODE_XML_FRAME_PITCH:
          this->m_motorSettings.setFramePitch(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_MOTOR_RADIUS:
          this->m_motorSettings.setMotorRadius(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_MOTOR_TYPE:
          value = xmlReadStringChild(doc, dataNode);
          if (strcmp(value.c_str(), "linear_motor") == 0) {
            this->m_motorSettings.setMotorType(usMotorSettings::LinearMotor);
          }
          else if (strcmp(value.c_str(), "tilting_motor") == 0) {
            this->m_motorSettings.setMotorType(usMotorSettings::TiltingMotor);
          }
          else if (strcmp(value.c_str(), "rotational_motor") == 0) {
            this->m_motorSettings.setMotorType(usMotorSettings::RotationalMotor);
          }
          else
            throw(vpException(vpException::fatalError, std::string("unknown image type in xml file")));
          value = "";
          break;
        case CODE_XML_ASSOCIATED_IMAGE_FILE_NAME:
          this->m_imageFileName = xmlReadStringChild(doc, dataNode);
          break;
        /*case CODE_XML_SEQUENCE_NAME:
          this->m_sequence_name = xmlReadStringChild(doc, dataNode);
          this->m_is_sequence = true;
          break;*/
        case CODE_XML_SEQUENCE_FRAME_RATE:
          this->m_sequence_frame_rate = xmlReadDoubleChild(doc, dataNode);
          this->m_is_sequence = true;
          break;
        case CODE_XML_SEQUENCE_FIRST_IMAGE_NUMBER:
          this->m_sequence_start = xmlReadIntChild(doc, dataNode);
          this->m_is_sequence = true;
          break;
        case CODE_XML_SEQUENCE_LAST_IMAGE_NUMBER:
          this->m_sequence_stop= xmlReadIntChild(doc, dataNode);
          this->m_is_sequence = true;
          break;
        default:
          vpTRACE("unknown tag in readConfigNode : %d, %s", iter_data->second, (iter_data->first).c_str());
          break;
        }
      }
    }
  }
}

/**
* Writing method, called by vpXmlParser::save().
* @param node : the root node of the document
*/
void 
usImageSettingsXmlParser::writeMainClass(xmlNodePtr node)
{
  std::string imageFileName = m_imageFileName;
  xmlWriteStringChild(node, "image_file_name", imageFileName);
  if (this->m_image_type == IMAGE_TYPE_RF) {
    xmlWriteStringChild(node, "image_type", std::string("rf"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_preScanSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_preScanSettings.getProbeRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_preScanSettings.isTransducerConvex());
    xmlWriteDoubleChild(node, "axial_resolution", m_preScanSettings.getAxialResolution());
  }
  if (this->m_image_type == IMAGE_TYPE_PRESCAN) {
    xmlWriteStringChild(node, "image_type", std::string("prescan"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_preScanSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_preScanSettings.getProbeRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_preScanSettings.isTransducerConvex());
    xmlWriteDoubleChild(node, "axial_resolution", m_preScanSettings.getAxialResolution());
  }
  else if (this->m_image_type == IMAGE_TYPE_POSTSCAN) {
    xmlWriteStringChild(node, "image_type", std::string("postscan"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_postScanSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_postScanSettings.getProbeRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_postScanSettings.isTransducerConvex());
    xmlWriteDoubleChild(node, "width_resolution", m_postScanSettings.getWidthResolution());
    xmlWriteDoubleChild(node, "height_resolution", m_postScanSettings.getHeightResolution());
  }
  if (m_is_3D) {
    xmlWriteDoubleChild(node, "frame_pitch", m_motorSettings.getFramePitch());
    xmlWriteDoubleChild(node, "motor_radius", m_motorSettings.getMotorRadius());
    if (m_motorSettings.getMotorType() == usMotorSettings::LinearMotor) {
      xmlWriteStringChild(node, "motor_type", std::string("linear_motor"));
    }
    else if (m_motorSettings.getMotorType() == usMotorSettings::TiltingMotor) {
      xmlWriteStringChild(node, "motor_type", std::string("tilting_motor"));
    }
    else if (m_motorSettings.getMotorType() == usMotorSettings::RotationalMotor) {
      xmlWriteStringChild(node, "motor_type", std::string("rotational_motor"));
    }
  }
  if (m_is_sequence) {
  std::cout << "writing sequence parameters" << std::endl;
    //xmlWriteStringChild(node, "sequence_name", this->m_sequence_name);
    xmlWriteDoubleChild(node, "sequence_frame_rate", this->m_sequence_frame_rate);
    xmlWriteIntChild(node, "sequence_start_number", this->m_sequence_start);
    xmlWriteIntChild(node, "sequence_stop_number", this->m_sequence_stop);
  }
}

/**
* Setter for pre-scan settings.
* @param imagePrescanSettings : settings to write in the xml file.
*/
void usImageSettingsXmlParser::setImagePreScanSettings(const usImagePreScanSettings imagePrescanSettings)
{
  m_preScanSettings = imagePrescanSettings;
}

/**
* Setter for post-scan settings.
* @param imagePostScanSettings : settings to write in the xml file.
*/
void usImageSettingsXmlParser::setImagePostScanSettings(const usImagePostScanSettings& imagePostScanSettings)
{
  m_postScanSettings = imagePostScanSettings;
}

/**
* Setter for pre-scan settings. Each transducer setting available.
* @param probeRadius : the probe rabius.
* @param scanLinePitch : the scanline pitch.
* @param isTransducerConvex : the transducer type (true if convex transducer, false if linear).
* @param axialResolution : the image axial resolution.
* @param image_type : image type (rf or pre-scan).
*/
void usImageSettingsXmlParser::setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double axialResolution, usImageType image_type) 
{
  if (image_type == usImageSettingsXmlParser::IMAGE_TYPE_PRESCAN || image_type == usImageSettingsXmlParser::IMAGE_TYPE_RF)
  {
    m_preScanSettings.setTransducerConvexity(isTransducerConvex);
    m_preScanSettings.setProbeRadius(probeRadius);
    m_preScanSettings.setScanLinePitch(scanLinePitch);
    m_preScanSettings.setAxialResolution(axialResolution);
    m_image_type = image_type;
  }
  else {
    throw(vpException(vpException::fatalError, "trying to write axial resolution in a image not rf nor pre-scan !"));
  }
}

/**
* Setter for post-scan settings. Each transducer setting available.
* @param probeRadius : the probe rabius.
* @param scanLinePitch : the scanline pitch.
* @param isTransducerConvex : the transducer type (true if convex transducer, false if linear).
* @param widthResolution : the image width resolution.
* @param heightResolution : the image height resolution.
*/
void usImageSettingsXmlParser::setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double widthResolution, double heightResolution)
{
  m_postScanSettings.setTransducerConvexity(isTransducerConvex);
  m_postScanSettings.setProbeRadius(probeRadius);
  m_postScanSettings.setScanLinePitch(scanLinePitch);
  m_postScanSettings.setHeightResolution(widthResolution);
  m_postScanSettings.setWidthResolution(heightResolution);
  m_image_type = usImageSettingsXmlParser::IMAGE_TYPE_POSTSCAN;
}

/**
* Setter for motor settings (3D images).
* @param motorSettings : motor settings to write in the xml file.
*/
void usImageSettingsXmlParser::setMotorSettings(const usMotorSettings &motorSettings)
{
  m_is_3D = true;
  m_motorSettings = motorSettings;
}

/**
* Setter for image file name associated to the settings wrote in the xml file.
* @param imageFileName : image file name with extestion.
*/
void usImageSettingsXmlParser::setImageFileName(std::string imageFileName)
{ 
  m_imageFileName = imageFileName;
}
#endif //VISP_HAVE_XML2
