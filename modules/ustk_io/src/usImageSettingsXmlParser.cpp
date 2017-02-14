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
 * @file usImageSettingsXmlParser.cpp
 * @brief Input/output operations between ultrasound image settings and the assiciated xml files.
 */

#include<visp3/ustk_io/usImageSettingsXmlParser.h>
#ifdef VISP_HAVE_XML2

 /**
 * Default constructor.
 */
usImageSettingsXmlParser::usImageSettingsXmlParser()
  : m_transducerSettings(usTransducerSettings()),
    m_spacingX(0.0), m_spacingY(0.0), m_spacingZ(0.0),
    m_motorSettings(usMotorSettings()), m_imageFileName(std::string("")),
    m_image_type(us::UNKNOWN), m_is_3D(false), m_is_sequence(false)
{
  nodeMap["settings"] = CODE_XML_SETTINGS;
  nodeMap["image_type"] = CODE_XML_IMAGE_TYPE;
  nodeMap["scanline_pitch"] = CODE_XML_SCANLINE_PITCH;
  nodeMap["probe_radius"] = CODE_XML_TRANSDUCER_RADIUS;
  nodeMap["is_probe_convex"] = CODE_XML_IS_TRANSDUCER_CONVEX;
  nodeMap["frame_pitch"] = CODE_XML_FRAME_PITCH;
  nodeMap["motor_radius"] = CODE_XML_MOTOR_RADIUS;
  nodeMap["motor_type"] = CODE_XML_MOTOR_TYPE;
  nodeMap["axial_resolution"] = CODE_XML_AXIAL_RESOLUTION;
  nodeMap["height_resolution"] = CODE_XML_HEIGHT_RESOLUTION;
  nodeMap["width_resolution"] = CODE_XML_WIDTH_RESOLUTION;
  nodeMap["scanline_number"] = CODE_XML_SCANLINE_NUMBER;
  nodeMap["frame_number"] = CODE_XML_FRAME_NUMBER;
  nodeMap["spacing_x"] = CODE_XML_SPACING_X;
  nodeMap["spacing_y"] = CODE_XML_SPACING_Y;
  nodeMap["spacing_z"] = CODE_XML_SPACING_Z;
  nodeMap["image_file_name"] = CODE_XML_ASSOCIATED_IMAGE_FILE_NAME;
  nodeMap["sequence_frame_rate"] = CODE_XML_SEQUENCE_FRAME_RATE;
  nodeMap["sequence_start_number"] = CODE_XML_SEQUENCE_FIRST_IMAGE_NUMBER;
  nodeMap["sequence_stop_number"] = CODE_XML_SEQUENCE_LAST_IMAGE_NUMBER;
}

/**
* Assignement operator.
* @param twinparser usImageSettingsXmlParser to assign.
*/
usImageSettingsXmlParser& usImageSettingsXmlParser::operator =(const usImageSettingsXmlParser& twinparser)
{
  m_transducerSettings = twinparser.getTransducerSettings();
  m_imageFileName = twinparser.getImageFileName();
  m_motorSettings = twinparser.getMotorSettings();
  m_image_type = twinparser.getImageType();
  m_is_3D = twinparser.isImage3D();
  m_widthResolution = twinparser.getWidthResolution();
  m_heightResolution = twinparser.getHeightResolution();
  m_axialResolution = twinparser.getAxialResolution();
  m_spacingX = twinparser.getSpacingX();
  m_spacingY = twinparser.getSpacingY();
  m_spacingZ = twinparser.getSpacingZ();

  return *this;
}

/**
* Destructor.
*/
usImageSettingsXmlParser::~usImageSettingsXmlParser()
{
  
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
            this->m_image_type = us::POSTSCAN_2D;
          }
          else if (strcmp(value.c_str(), "prescan") == 0) {
            this->m_image_type = us::PRESCAN_2D;
          }
          else if (strcmp(value.c_str(), "rf") == 0) {
            this->m_image_type = us::RF_2D;
          }
          else
            throw(vpException(vpException::fatalError, std::string("unknown image type in xml file")));
          break;
        case CODE_XML_AXIAL_RESOLUTION:
          if (this->m_image_type == us::PRESCAN_2D || this->m_image_type == us::RF_2D) {
            this->m_axialResolution =  xmlReadDoubleChild(doc, dataNode);
          }
          else 
            throw(vpException(vpException::fatalError, std::string("Trying to assign an axial resolution to a post-scan image !")));
          break;
        case CODE_XML_HEIGHT_RESOLUTION:
          if (this->m_image_type == us::RF_2D || this->m_image_type == us::PRESCAN_2D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an height resolution to a pre-scan image !")));
          }
          else if(this->m_is_3D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an height resolution to a 3D image ! Use spacing_y instead.")));
          }
          else 
            this->m_heightResolution = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_WIDTH_RESOLUTION :
          if (this->m_image_type == us::RF_2D || this->m_image_type == us::PRESCAN_2D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an width resolution to a pre-scan image !")));
          }
          else if(this->m_is_3D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an height resolution to a 3D image ! Use spacing_x instead.")));
          }
          else
            this->m_widthResolution = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_SPACING_X :
          if (this->m_image_type == us::RF_2D || this->m_image_type == us::PRESCAN_2D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign a spacing to a pre-scan image !")));
          }
          else if(!this->m_is_3D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign a spacing to a 2D image ! Use height/width resolutions instead.")));
          }
          else
            this->m_widthResolution = xmlReadDoubleChild(doc, dataNode);
          break;
               case CODE_XML_SPACING_Y :
          if (this->m_image_type == us::RF_2D || this->m_image_type == us::PRESCAN_2D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign a spacing to a pre-scan image !")));
          }
          else if(!this->m_is_3D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an spacing to a 2D image ! Use height/width resolutions instead.")));
          }
          else
            this->m_widthResolution = xmlReadDoubleChild(doc, dataNode);
          break;
                case CODE_XML_SPACING_Z :
          if (this->m_image_type == us::RF_2D || this->m_image_type == us::PRESCAN_2D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign a spacing to a pre-scan image !")));
          }
          else if(!this->m_is_3D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign an spacing to a 2D image ! Use height/width resolutions instead.")));
          }
          else
            this->m_widthResolution = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_SCANLINE_NUMBER :
          if (this->m_image_type == us::RF_2D || this->m_image_type == us::PRESCAN_2D) {
            throw(vpException(vpException::fatalError, std::string("Trying to assign a scan line number to a pre-scan image (for pre-scan images scan line number is the image width) !")));
          } else
            this->m_transducerSettings.setScanLineNumber(xmlReadIntChild(doc, dataNode));
          break;
        case CODE_XML_SCANLINE_PITCH:
          this->m_transducerSettings.setScanLinePitch(xmlReadDoubleChild(doc, dataNode));
          this->m_transducerSettings.setScanLinePitch(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_TRANSDUCER_RADIUS:
          this->m_transducerSettings.setTransducerRadius(xmlReadDoubleChild(doc, dataNode));
          this->m_transducerSettings.setTransducerRadius(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_IS_TRANSDUCER_CONVEX:
          this->m_transducerSettings.setTransducerConvexity(xmlReadBoolChild(doc, dataNode));
          this->m_transducerSettings.setTransducerConvexity(xmlReadBoolChild(doc, dataNode));
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
  if (this->m_image_type == us::RF_2D) {
    xmlWriteStringChild(node, "image_type", std::string("rf"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_transducerSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_transducerSettings.getTransducerRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_transducerSettings.isTransducerConvex());
    xmlWriteDoubleChild(node, "axial_resolution", m_axialResolution);
  }
  if (this->m_image_type == us::PRESCAN_2D) {
    xmlWriteStringChild(node, "image_type", std::string("prescan"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_transducerSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_transducerSettings.getTransducerRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_transducerSettings.isTransducerConvex());
    xmlWriteDoubleChild(node, "axial_resolution", m_axialResolution);
  }
  else if (this->m_image_type == us::POSTSCAN_2D) {
    xmlWriteStringChild(node, "image_type", std::string("postscan"));
    xmlWriteDoubleChild(node, "scanline_pitch", m_transducerSettings.getScanLinePitch());
    xmlWriteDoubleChild(node, "probe_radius", m_transducerSettings.getTransducerRadius());
    xmlWriteBoolChild(node, "is_probe_convex", m_transducerSettings.isTransducerConvex());
    xmlWriteIntChild(node, "scanline_number", m_transducerSettings.getScanLineNumber());
    if(m_is_3D) {
      xmlWriteIntChild(node, "frame_number", m_motorSettings.getFrameNumber());
      xmlWriteDoubleChild(node, "spacing_x", m_spacingX);
      xmlWriteDoubleChild(node, "spacing_y", m_spacingY);
      xmlWriteDoubleChild(node, "spacing_z", m_spacingZ);
    }
    else {
      xmlWriteDoubleChild(node, "height_resolution", m_heightResolution);
      xmlWriteDoubleChild(node, "width_resolution", m_widthResolution);
    }
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
* Setter for pre-scan settings. Each transducer setting available.
* @param transducerRadius : the Transducer rabius.
* @param scanLinePitch : the scan line pitch.
* @param isTransducerConvex : the transducer type (true if convex transducer, false if linear).
* @param axialResolution : the image axial resolution.
* @param image_type : image type (rf or pre-scan).
*/
void usImageSettingsXmlParser::setImageSettings(double transducerRadius, double scanLinePitch, bool isTransducerConvex,
                                                double axialResolution, us::ImageType image_type)
{
  if (image_type == us::PRESCAN_2D || image_type == us::RF_2D)
  {
    m_transducerSettings.setTransducerConvexity(isTransducerConvex);
    m_transducerSettings.setTransducerRadius(transducerRadius);
    m_transducerSettings.setScanLinePitch(scanLinePitch);
    m_axialResolution = axialResolution;
    m_image_type = image_type;
  }
  else {
    throw(vpException(vpException::fatalError, "trying to write axial resolution in a image not rf nor pre-scan !"));
  }
}

/**
* Setter for post-scan settings. Each transducer setting available.
* @param transducerRadius : the transducer rabius.
* @param scanLinePitch : the scan line pitch.
* @param isTransducerConvex : the transducer type (true if convex transducer, false if linear).
* @param scanLineNumber : the number of scan lines of the probe used.
* @param widthResolution : the image width resolution.
* @param heightResolution : the image height resolution.
*/
void usImageSettingsXmlParser::setImageSettings(double transducerRadius, double scanLinePitch, bool isTransducerConvex, unsigned int scanLineNumber,
                                                double widthResolution, double heightResolution)
{
  m_transducerSettings.setTransducerConvexity(isTransducerConvex);
  m_transducerSettings.setTransducerRadius(transducerRadius);
  m_transducerSettings.setScanLinePitch(scanLinePitch);
  m_transducerSettings.setScanLineNumber(scanLineNumber);
  m_heightResolution = widthResolution;
  m_widthResolution = heightResolution;
  m_image_type = us::POSTSCAN_2D;
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
void usImageSettingsXmlParser::setImageFileName(const std::string &imageFileName)
{ 
  m_imageFileName = imageFileName;
}
#endif //VISP_HAVE_XML2
