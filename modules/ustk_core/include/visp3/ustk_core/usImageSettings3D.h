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
 * @file usImageSettings3D.h
 * @brief Generic ultrasound 3D image settings.
 */

#ifndef US_IMAGE_SETTINGS_3D_H
#define US_IMAGE_SETTINGS_3D_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usImageSettings.h>

/**
 * @class usImageSettings3D
 * @brief Generic class for 3D ultrasound data : storage of probe settings.
 *
 * This class represents a ultrasound image settings.
 */
class VISP_EXPORT usImageSettings3D : public usImageSettings {
public:
  usImageSettings3D();
  
  usImageSettings3D(double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isImageConvex, bool isMotorConvex);

  usImageSettings3D(const usImageSettings3D &other);

  virtual ~usImageSettings3D();

  usImageSettings3D& operator=(const usImageSettings3D& other);

  bool operator==(const usImageSettings3D& other);
  
  //Settings form the 3D probe

  void setMotorRadius(double motorRadius);
  
  double getMotorRadius() const;

  void setFramePitch(double framePitch);

  double getFramePitch() const;

  void setMotorConvex(bool isMotorConvex);

  bool isMotorConvex() const;

  virtual void printProbeSettings();

private:
  //Settings from the 3D probe
  double m_motorRadius;
  double m_framePitch;
  bool m_isMotorConvex;
};

#endif // US_IMAGE_SETTINGS_3D_H
