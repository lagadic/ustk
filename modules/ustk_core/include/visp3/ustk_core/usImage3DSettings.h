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
 * @file usImage3DSettings.h
 * @brief Generic ultrasound 3D image settings.
 */

#ifndef US_IMAGE_3D_SETTINGS_H
#define US_IMAGE_3D_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usImageSettings.h>

/**
 * @class usImage3DSettings
 *
 * @brief Generic class for 3D ultrasound data common settings associated to the 3D probe used
 * during acquisition.
 *
 * This class represents common 3D ultrasound image settings which are:
 * - the common settings implemented in usImageSettings corresponding to the transducer settings.
 *   We recall that these common settings are:
 *   - the type of ultrasound transducer used for data acquisition: convex or linear
 *   - the transducer radius in meters (value set to zero for a linear transducer)
 *   - the scan line pitch that corresponds to the angle (in radians) between
 *     to successive scan lines beams when the transducer is convex, or to the distance (in meters)
 *     when the transducer is linear. See usImageSettings description for more details.
 *   .
 * - the common settings corresponding to the motor used to procude the third dimension.
 *   These settings are:
 *   - the type of motor used to move the transducer: convex or linear
 *   - the motor radius (value set to zero for a linear motor)
 *   - the frame pitch that corresponds to the angle (in radians) between
 *     to successive data acquisitions when the motor is convex, or to the distance (in meters)
 *     when the motor is linear.
 *   .
 *
 */
class VISP_EXPORT usImage3DSettings : public usImageSettings {
public:
  usImage3DSettings();
  usImage3DSettings(double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isImageConvex, bool isMotorRotating);
  usImage3DSettings(const usImage3DSettings &other);
  virtual ~usImage3DSettings();

  /** @name Inherited functionalities from usImage3DSettings */
  //@{
  double getFramePitch() const;
  double getMotorRadius() const;
  bool isMotorRotating() const;

  usImage3DSettings& operator=(const usImage3DSettings& other);
  bool operator==(const usImage3DSettings& other);
  
  virtual void printProbeSettings();

  // Settings from the 3D probe
  void setMotorRadius(double motorRadius);
  void setFramePitch(double framePitch);
  void setMotorConvexity(bool isMotorRotating);

  //@}

private:
  //Settings from the 3D probe
  double m_motorRadius;
  double m_framePitch;
  bool m_isMotorRotating;
};

#endif // US_IMAGE_3D_SETTINGS_H
