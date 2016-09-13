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
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usImageSettings.h
 * @brief Generic ultrasound data.
 * @author Pierre Chatelain
 */

#ifndef US_IMAGE_SETTINGS_H
#define US_IMAGE_SETTINGS_H

 //std includes
#include <iostream>

 //visp includes
#include <visp3/core/vpConfig.h>

 //ustk includes


/**
 * Ultasound scanner type.
 *
 * Defines the ultrasound scanner used for acquisition.
 */
enum usScannerType {
  UNKNOWN_SCANNER, SONIX_RP, SONIX_TOUCH, SONOSITE
};

/**
 * Ultasound probe type.
 *
 * Defines the ultrasound probe used for acquisition.
 *
 * @todo Add more probe types.
 */
enum usProbeType {
  UNKNOWN_PROBE, US_4DC7, SS_C60
};


/**
 * @class usImageSettings
 * @brief Generic class for ultrasound data : storage of probe and scanner settings.
 * @author Pierre Chatelain
 *
 * This class represents a generic ultrasound image.
 */
class VISP_EXPORT usImageSettings {
 public:
  usImageSettings();
  
  usImageSettings(float probeRadius,
				  float lineAngle,
				  float resolution,
				  float BSampleFreq,
				  float probeElementPitch);

  usImageSettings(const usImageSettings &other);

  virtual ~usImageSettings();

  usImageSettings& operator=(const usImageSettings& other);

  // Getters/Setters for general image informations

  void setScannerType(usScannerType scannerType);
  
  usScannerType getScannerType() const;
  
  void setProbeType(usProbeType probeType);

  usProbeType getProbeType() const;

  void setDataIdx(unsigned int idx);

  unsigned int getDataIdx() const;

  void setTimestamp(double timestamp);

  double getTimestamp() const;

  void setOriginX(double originX);

  double getOriginX() const;

  void setOriginY(double originY);

  double getOriginY() const;

  void setOrigin(double originX, double originY);
  
  //Settings form the probe

  void setProbeRadius(float probeRadius);
  
  float getProbeRadius() const;

  void setLineAngle(float angle);

  float getLineAngle() const;

  void setResolution(float resolution);

  float getResolution() const;

  void setBSampleFreq(float freq);

  float getBSampleFreq() const;

  float getProbeElementPitch() const;

  void setProbeElementPitch(float probeElementPitch);

  virtual void printInfo();
  
  virtual void printProbeSettings();

 private:
  //general settings
  usScannerType m_scannerType;
  usProbeType m_probeType;
  unsigned int m_dataIdx;
  double m_timestamp;
  double m_originX;
  double m_originY;
  //double m_originZ; not in 2D
  
  //Settings from the probe
  float m_probeRadius;
  float m_lineAngle;
  float m_resolution;
  float m_BSampleFreq;
  float m_probeElementPitch;
};

#endif // US_IMAGE_SETTINGS_H
