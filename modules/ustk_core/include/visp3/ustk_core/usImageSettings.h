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

#include <visp3/core/vpConfig.h>

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
  /**
   * Constructor.
   */
  usImageSettings();
  
  /**
   * Constructor.
   */
  usImageSettings(float probeRadius,
				  float lineAngle,
				  float resolution,
				  float BSampleFreq,
				  float probeElementPitch);

  /**
   * Destructor.
   */
  virtual ~usImageSettings();

  /**
   * Assignment operator.
   */
  usImageSettings& operator=(const usImageSettings& other);

  /**
   * Set the scanner type (SONIX_RP, SONIX_TOUCH, SONOSITE).
   */
  void setScannerType(usScannerType scannerType);
  
  /**
   * Get the scanner type.
   */
  usScannerType getScannerType() const;
  
  /**
   * Set the probe type (US_4DC7, SS_C60).
   */
  void setProbeType(usProbeType probeType);

  /**
   * Get the probe type.
   */
  usProbeType getProbeType() const;

  /**
   * Set the data index.
   */
  void setDataIdx(unsigned int idx);

  /**
   * Get the data index.
   */
  unsigned int getDataIdx() const;

  /**
   * Set the data timestamp.
   */
  void setTimestamp(double timestamp);

  /**
   * Set the data timestamp.
   */
  double getTimestamp() const;

  /**
   * Set the x-coordinate of the volume origin.
   */
  void setOriginX(double originX);

  /**
   * Get the x-coordinate of the volume origin.
   */
  double getOriginX() const;

  /**
   * Set the y-coordinate of the volume origin.
   */
  void setOriginY(double originY);

  /**
   * Get the y-coordinate of the volume origin.
   */
  double getOriginY() const;

  /*Not for 2D version
   * Set the z-coordinate of the volume origin.
   /
  void setOriginZ(double originZ);

  /**
   * Get the z-coordinate of the volume origin.
   /
  double getOriginZ() const;*/

  /**
   * Set the coordinates of the image origin.
   */
  void setOrigin(double originX, double originY);
  
  //Settings form the probe

  /**
   * Set the probe  radius (m).
   */
  void setProbeRadius(float probeRadius);
  
  /**
   * Get the probe radius (m).
   */
  float getProbeRadius() const;

  /**
   * Get the line angle (rad).
   */
  void setLineAngle(double angle);

  /**
   * Get the line angle (rad).
   */
  float getLineAngle() const;

  /**
   * Get the linear resolution (m).
   */
  void setResolution(float resolution);

  /**
   * Get the linear resolution (m).
   */
  float getResolution() const;

  /**
   * Get the B-sample frequence (Hz).
   */
  void setBSampleFreq(double freq);

  /**
   * Get the B-sample frequence (Hz).
   */
  float getBSampleFreq() const;

  /**
   * Get the probe element pitch.
   */
  float getProbeElementPitch() const;

  /**
   * Set the probe element pitch.
   */
  void setProbeElementPitch(float probeElementPitch);

  /**
   * Print data information.
   */
  virtual void printInfo();
  
  /**
   * Print probe settings information.
   */
  virtual void printProbeSettings();

 protected:
  //general settings
  usScannerType m_scannerType;
  usProbeType m_probeType;
  unsigned int m_dataIdx;
  double m_timestamp;
  double m_originX;
  double m_originY;
  //double m_originZ; not in 2D
  
  //Settings form the probe
  float m_probeRadius;
  float m_lineAngle;
  float m_resolution;
  float m_BSampleFreq;
  float m_probeElementPitch;

  //probe settings verification, turn it to true only when you are shure that your instance of usImageXXXXX contains the good probe settings
  bool probeSettingsAreValid; //true if valid
};

#endif // US_IMAGE_SETTINGS_H
