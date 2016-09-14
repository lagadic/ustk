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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImageSettings.h
 * @brief Generic ultrasound image settings.
 */

#ifndef US_IMAGE_SETTINGS_H
#define US_IMAGE_SETTINGS_H

 //std includes
#include <iostream>

 //visp includes
#include <visp3/core/vpConfig.h>

 //ustk includes

/**
 * @class usImageSettings
 * @brief Generic class for ultrasound data : storage of probe and scanner settings.
 *
 * This class represents a ultrasound image settings.
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
  
  virtual void printProbeSettings();

 private:  
  //Settings from the probe
  float m_probeRadius;
  float m_lineAngle;
  float m_resolution;
  float m_BSampleFreq;
  float m_probeElementPitch;
};

#endif // US_IMAGE_SETTINGS_H
