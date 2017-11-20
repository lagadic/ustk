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
 * @file usImageHeaderXmlParser.h
 * @brief Input/output operations between ultrasound image header and the assiciated xml files. Used for virtual server.
 */

#ifndef __usImageHeaderXmlParser_h_
#define __usImageHeaderXmlParser_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_XML2) && (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT))

#include <iostream>
#include <string>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParser.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/ustk_core/us.h>

/**
 * @class usImageHeaderXmlParser
 * @brief Input/output operations between ultrasound image header and the assiciated xml files. Used for virtual server.
 * @ingroup module_ustk_grabber
 */
class VISP_EXPORT usImageHeaderXmlParser : public vpXmlParser
{
public:
  usImageHeaderXmlParser();
  virtual ~usImageHeaderXmlParser();

  typedef enum {
    CODE_XML_BAD = -1,
    CODE_XML_FRAME_COUNT,
    CODE_XML_TIME_STAMP,
    CODE_XML_DATA_RATE,
    CODE_XML_DATA_LENGTH,
    CODE_XML_SAMPLE_SIZE,
    CODE_XML_IMAGE_TYPE,
    CODE_XML_FRAME_WITH,
    CODE_XML_FRAME_HEIGHT,
    CODE_XML_PIXEL_WIDTH,
    CODE_XML_PIXEL_HEIGHT,
    CODE_XML_TRANSMIT_FRQUENCY,
    CODE_XML_SAMPLING_FREQUENCY,
    CODE_XML_TRANSDUCER_RADIUS,
    CODE_XML_SCANLINE_PITCH,
    CODE_XML_SCANLINE_NUMBER,
    CODE_XML_IMAGE_DEPTH,
    CODE_XML_ANGLE_PER_FRAME,
    CODE_XML_FRAME_PER_VOLUME,
    CODE_XML_MOTOR_RADIUS,
    CODE_XML_MOTOR_TYPE,
    CODE_XML_SEQUENCE_NAME,
  } vpXmlCodeType;

  typedef enum { SEQUENCE_OK, SEQUENCE_ERROR } vpXmlCodeSequenceType;

  // getters
  us::usImageHeader const getImageHeader() { return m_imageHeader; }
  std::string getSequenceFileName() const { return m_sequence_name; }

  // Data setters
  void setImageHeader(const us::usImageHeader imageHeader) { m_imageHeader = imageHeader; }
  void setSequenceFileName(const std::string sequence_name) { m_sequence_name = sequence_name; }

private:
  uint32_t xmlReadUInt32Child(xmlDocPtr doc, xmlNodePtr node);
  uint64_t xmlReadUInt64Child(xmlDocPtr doc, xmlNodePtr node);
  void xmlWriteUInt32Child(xmlNodePtr node, const char *label, const uint32_t value);
  void xmlWriteUInt64Child(xmlNodePtr node, const char *label, const uint64_t value);

  us::usImageHeader m_imageHeader;
  std::string m_sequence_name;

protected:
  void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void writeMainClass(xmlNodePtr node);
};
#endif // US_IMAGE_HEADER_XML_PARSER_H
#endif // VISP_HAVE_XML2 || QT
