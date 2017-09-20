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
 * @file usImageHeaderXmlParser.cpp
 * @brief Input/output operations between ultrasound image settings and the assiciated xml files.
 */

#include<visp3/ustk_grabber/usImageHeaderXmlParser.h>
#ifdef VISP_HAVE_XML2

 /**
 * Default constructor.
 */
usImageHeaderXmlParser::usImageHeaderXmlParser()
  : m_imageHeader(), m_sequence_name()
{
  nodeMap["frame_count"] = CODE_XML_FRAME_COUNT;
  nodeMap["time_stamp"] = CODE_XML_TIME_STAMP;
  nodeMap["data_rate"] = CODE_XML_DATA_RATE;
  nodeMap["data_length"] = CODE_XML_DATA_LENGTH;
  nodeMap["sample_size"] = CODE_XML_SAMPLE_SIZE;
  nodeMap["image_type"] = CODE_XML_IMAGE_TYPE;
  nodeMap["frame_with"] = CODE_XML_FRAME_WITH;
  nodeMap["frame_height"] = CODE_XML_FRAME_HEIGHT;
  nodeMap["pixel_width"] = CODE_XML_PIXEL_WIDTH;
  nodeMap["pixel_height"] = CODE_XML_PIXEL_HEIGHT;
  nodeMap["transmit_frequency"] = CODE_XML_TRANSMIT_FRQUENCY;
  nodeMap["sampling_frequency"] = CODE_XML_SAMPLING_FREQUENCY;
  nodeMap["transducer_radius"] = CODE_XML_TRANSDUCER_RADIUS;
  nodeMap["scanline_pitch"] = CODE_XML_SCANLINE_PITCH;
  nodeMap["scanline_number"] = CODE_XML_SCANLINE_NUMBER;
  nodeMap["image_depth"] = CODE_XML_IMAGE_DEPTH;
  nodeMap["angle_per_frame"] = CODE_XML_ANGLE_PER_FRAME;
  nodeMap["frame_per_volume"] = CODE_XML_FRAME_PER_VOLUME;
  nodeMap["motor_radius"] = CODE_XML_MOTOR_RADIUS;
  nodeMap["motor_type"] = CODE_XML_MOTOR_TYPE;
  nodeMap["sequence_name"] = CODE_XML_SEQUENCE_NAME;
}

/**
* Destructor.
*/
usImageHeaderXmlParser::~usImageHeaderXmlParser()
{
  
}

/**
* Reading method, called by vpXmlParser::parse().
* @param doc a pointer representing the document
* @param node : the root node of the document
*/
void
usImageHeaderXmlParser::readMainClass (xmlDocPtr doc, xmlNodePtr node)
{
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case CODE_XML_FRAME_COUNT:
          this->m_imageHeader.frameCount = xmlReadUInt32Child(doc, dataNode);
          break;
        case CODE_XML_TIME_STAMP:
           this->m_imageHeader.timeStamp = xmlReadUInt64Child(doc, dataNode);
           break;
        case CODE_XML_DATA_RATE:
          this->m_imageHeader.dataRate = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_DATA_LENGTH:
          this->m_imageHeader.dataLength = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_SAMPLE_SIZE:
          this->m_imageHeader.ss = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_IMAGE_TYPE:
          this->m_imageHeader.imageType = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_FRAME_WITH:
          this->m_imageHeader.frameWidth = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_FRAME_HEIGHT:
          this->m_imageHeader.frameHeight = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_PIXEL_WIDTH:
          this->m_imageHeader.pixelWidth = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_PIXEL_HEIGHT:
          this->m_imageHeader.pixelHeight = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_TRANSMIT_FRQUENCY:
          this->m_imageHeader.transmitFrequency = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_SAMPLING_FREQUENCY:
          this->m_imageHeader.samplingFrequency = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_TRANSDUCER_RADIUS:
          this->m_imageHeader.transducerRadius = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_SCANLINE_PITCH:
          this->m_imageHeader.scanLinePitch = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_SCANLINE_NUMBER:
          this->m_imageHeader.scanLineNumber = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_IMAGE_DEPTH:
          this->m_imageHeader.imageDepth = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_ANGLE_PER_FRAME:
          this->m_imageHeader.anglePerFr = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_FRAME_PER_VOLUME:
          this->m_imageHeader.framesPerVolume = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_MOTOR_RADIUS:
          this->m_imageHeader.motorRadius = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_MOTOR_TYPE:
          this->m_imageHeader.motorType = xmlReadIntChild(doc, dataNode);
          break;
        case CODE_XML_SEQUENCE_NAME:
          this->m_sequence_name = xmlReadStringChild(doc, dataNode);
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
usImageHeaderXmlParser::writeMainClass(xmlNodePtr node)
{
  xmlWriteUInt32Child(node, "frame_count", this->m_imageHeader.frameCount);
  xmlWriteUInt64Child(node, "time_stamp", this->m_imageHeader.timeStamp);
  xmlWriteDoubleChild(node, "time_rate", this->m_imageHeader.dataRate);
  xmlWriteIntChild(node, "data_length", this->m_imageHeader.dataLength);
  xmlWriteIntChild(node, "sample_size", this->m_imageHeader.ss);
  xmlWriteIntChild(node, "image_type", this->m_imageHeader.imageType);
  xmlWriteIntChild(node, "frame_with", this->m_imageHeader.frameWidth);
  xmlWriteIntChild(node, "frame_height", this->m_imageHeader.frameHeight);
  xmlWriteDoubleChild(node, "pixel_width", this->m_imageHeader.pixelWidth);
  xmlWriteDoubleChild(node, "pixel_height", this->m_imageHeader.pixelHeight);
  xmlWriteIntChild(node, "transmit_frequency", this->m_imageHeader.transmitFrequency);
  xmlWriteIntChild(node, "sampling_frequency", this->m_imageHeader.samplingFrequency);
  xmlWriteDoubleChild(node, "transducer_radius", this->m_imageHeader.transducerRadius);
  xmlWriteDoubleChild(node, "scanline_pitch", this->m_imageHeader.scanLinePitch);
  xmlWriteUnsignedIntChild(node, "scanline_number", this->m_imageHeader.scanLineNumber);
  xmlWriteIntChild(node, "image_depth", this->m_imageHeader.imageDepth);
  xmlWriteDoubleChild(node, "angle_per_frame", this->m_imageHeader.anglePerFr);
  xmlWriteIntChild(node, "frame_per_volume", this->m_imageHeader.framesPerVolume);
  xmlWriteDoubleChild(node, "motor_radius", this->m_imageHeader.motorRadius);
  xmlWriteIntChild(node, "motor_type", this->m_imageHeader.motorType);
  xmlWriteStringChild(node, "sequence_name", this->m_sequence_name);
}

quint32 usImageHeaderXmlParser::xmlReadUInt32Child(xmlDocPtr doc, xmlNodePtr node) {
 if(node ->xmlChildrenNode == NULL){
    std::string errorMsg = "Empty node " + std::string((char*)node->name) + ", cannot read int";
    std::cerr << errorMsg << std::endl;
    throw vpException(vpException::fatalError, errorMsg);
  }
  char * val_char;
  char * control_convert;
  quint32 val_int;

  val_char = (char *) xmlNodeListGetString(doc, node ->xmlChildrenNode, 1);
  val_int = (quint32)strtol ((char *)val_char, &control_convert, 10);

  if (val_char == control_convert){
    xmlFree((xmlChar*) val_char);
    throw vpException(vpException::ioError, "cannot parse entry to quint32");
  }
  xmlFree((xmlChar*) val_char);

  return val_int;
}

quint64 usImageHeaderXmlParser::xmlReadUInt64Child(xmlDocPtr doc, xmlNodePtr node) {
 if(node ->xmlChildrenNode == NULL){
    std::string errorMsg = "Empty node " + std::string((char*)node->name) + ", cannot read int";
    std::cerr << errorMsg << std::endl;
    throw vpException(vpException::fatalError, errorMsg);
  }
  char * val_char;
  char * control_convert;
  quint64 val_int;

  val_char = (char *) xmlNodeListGetString(doc, node ->xmlChildrenNode, 1);
  val_int = (quint64)strtouq ((char *)val_char, &control_convert, 10);

  if (val_char == control_convert){
    xmlFree((xmlChar*) val_char);
    throw vpException(vpException::ioError, "cannot parse entry to quint64");
  }
  xmlFree((xmlChar*) val_char);

  return val_int;
}

void usImageHeaderXmlParser::xmlWriteUInt32Child(xmlNodePtr node, const char* label, const quint32 value)
{
  xmlNodePtr tmp = xmlNewChild(node, NULL, (xmlChar*)label, (xmlChar*)QString::number(value).toStdString().c_str());
  xmlAddChild(node, tmp);
}

void usImageHeaderXmlParser::xmlWriteUInt64Child(xmlNodePtr node, const char* label, const quint64 value)
{
  xmlNodePtr tmp = xmlNewChild(node, NULL, (xmlChar*)label, (xmlChar*)QString::number(value).toStdString().c_str());
  xmlAddChild(node, tmp);
}
#endif //VISP_HAVE_XML2
