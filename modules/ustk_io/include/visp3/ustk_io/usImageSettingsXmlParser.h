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

#ifndef US_IMAGE_SETTINGS_XML_PARSER_H
#define US_IMAGE_SETTINGS_XML_PARSER_H

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <iostream>
#include <visp3/ustk_core/usTransducerSettings.h>
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
 * @ingroup module_ustk_io
 */
class VISP_EXPORT usImageSettingsXmlParser: public vpXmlParser
{
public:
  usImageSettingsXmlParser();
  usImageSettingsXmlParser(usImageSettingsXmlParser& twinParser);
  usImageSettingsXmlParser& operator =(const usImageSettingsXmlParser& twinparser);
  virtual ~usImageSettingsXmlParser();

  typedef enum{
    CODE_XML_BAD = -1,
    CODE_XML_SETTINGS,
    CODE_XML_IMAGE_TYPE,
    CODE_XML_SCANLINE_PITCH,
    CODE_XML_PROBE_RADIUS,
    CODE_XML_IS_PROBE_CONVEX,
    CODE_XML_FRAME_PITCH,
    CODE_XML_MOTOR_RADIUS, 
    CODE_XML_MOTOR_TYPE,
    CODE_XML_AXIAL_RESOLUTION,
    CODE_XML_HEIGHT_RESOLUTION,
    CODE_XML_WIDTH_RESOLUTION,
    CODE_XML_SCANLINE_NUMBER,
    CODE_XML_FRAME_NUMBER,
    CODE_XML_SPACING_X,
    CODE_XML_SPACING_Y,
    CODE_XML_SPACING_Z,

    CODE_XML_ASSOCIATED_IMAGE_FILE_NAME,
    CODE_XML_SEQUENCE_FRAME_RATE,
    CODE_XML_SEQUENCE_FIRST_IMAGE_NUMBER,
    CODE_XML_SEQUENCE_LAST_IMAGE_NUMBER,
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
    IMAGE_TYPE_POSTSCAN,
  } usImageType;


  //getters
  double getAxialResolution() const { return m_axialResolution; }
  double getFrameNumber() const { return m_frameNumber; }
  std::string getImageFileName() const {return m_imageFileName;}
  double getHeightResolution() const {return m_heightResolution;}
  usImageType getImageType() const { return m_image_type; }
  usMotorSettings getMotorSettings() const { return m_motorSettings; }
  unsigned int getScanLineNumber() const {return m_scanLineNumber;}
  double getSequenceFrameRate() const {return m_sequence_frame_rate;}
  int getSequenceStartNumber() const {return m_sequence_start;}
  int getSequenceStopNumber() const {return m_sequence_stop;}
  int getSpacingX() const {return m_spacingX;}
  int getSpacingY() const {return m_spacingY;}
  int getSpacingZ() const {return m_spacingZ;}
  double getWidthResolution() const { return m_widthResolution; }
  usTransducerSettings getTransducerSettings() const { return m_transducerSettings; }

  bool isImage3D() const { return m_is_3D; }
  bool isSequence() const {return m_is_sequence;}
  
  //Data setters
  void setFrameNumber(unsigned int frameNumber) {m_frameNumber = frameNumber;}
  void setImageFileName(const std::string &imageFileName);
  void setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double axialResolution, usImageType image_type);
  void setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, unsigned int scanLineNumber,
                        double widthResolution, double heightResolution);
  void setImageType(usImageType image_type) { m_image_type = image_type;}
  void setMotorSettings(const usMotorSettings &motorSettings);
  void setScanLineNumber(unsigned int scanLineNumber) {m_scanLineNumber = scanLineNumber;}
  void setSequenceFrameRate(double sequenceFrameRate) {m_sequence_frame_rate=sequenceFrameRate; m_is_sequence=true;}
  void setSequenceStartNumber(int sequenceStartNumber) {m_sequence_start = sequenceStartNumber; m_is_sequence=true;}
  void setSequenceStopNumber(int sequenceStopNumber) {m_sequence_stop = sequenceStopNumber; m_is_sequence=true;}
  void setSequenceType(bool isSequence) {m_is_sequence = isSequence;}
  void setSpacing(double spacingX, double spacingY, double spacingZ) {m_spacingX=spacingX;
                                                                      m_spacingY=spacingY;
                                                                      m_spacingZ=spacingZ;}
  void setTransducerSettings(const usTransducerSettings transducerSettings);

private:
  //basic 
  usTransducerSettings m_transducerSettings;

  //for 2D post scan images
  double m_widthResolution;
  double m_heightResolution;
  unsigned int m_scanLineNumber; //for 3D too

  //for 3D post scan images
  double m_spacingX;
  double m_spacingY;
  double m_spacingZ;
  unsigned int m_frameNumber;

  //for rf / pre-scan
  double m_axialResolution;

  //for 3D
  usMotorSettings m_motorSettings;

  std::string m_imageFileName;

  //to manage different resolution types
  usImageType m_image_type;
  bool m_is_3D;
  //to manage usltrasound images sequences
  bool m_is_sequence;
  double m_sequence_frame_rate;
  std::string m_sequence_name;
  int m_sequence_start;
  int m_sequence_stop;

protected:
   void readMainClass (xmlDocPtr doc, xmlNodePtr node);
   void writeMainClass (xmlNodePtr node);
};
#endif //US_IMAGE_SETTINGS_XML_PARSER_H
#endif //VISP_HAVE_XML2

