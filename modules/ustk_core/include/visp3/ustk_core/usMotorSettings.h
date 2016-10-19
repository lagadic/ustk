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
 * @file usMotorSettings.h
 * @brief Generic ultrasound probe motor settings.
 */

#ifndef US_MOTOR_SETTINGS_H
#define US_MOTOR_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usTransducerSettings.h>

/**
 * @class usMotorSettings
 *
 * @brief Generic class for 3D ultrasound motor settings associated to the 3D probe used
 * during acquisition.
 * @ingroup module_ustk_core
 *
 *
 * This class represents motor settings for 3D ultrasound images which are:
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
class VISP_EXPORT usMotorSettings {
public:
  //enum for motor rotation type
  typedef enum {
    LinearMotor = 0,
    TiltingMotor,
    RotationalMotor
  } usMotorType;

  usMotorSettings();
  usMotorSettings(double motorRadius, double framePitch, usMotorType m_motorType);
  usMotorSettings(const usMotorSettings &other);
  virtual ~usMotorSettings();

  /** @name Inherited functionalities from usMotorSettings */
  //@{
  double getFramePitch() const;
  double getMotorRadius() const;
  usMotorType getMotorType() const;

  usMotorSettings& operator=(const usMotorSettings& other);
  bool operator==(const usMotorSettings& other);
  friend VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usMotorSettings& other);

  // Settings from the 3D probe
  void setMotorRadius(double motorRadius);
  void setFramePitch(double framePitch);
  void setMotorType(usMotorType m_motorType);
  void setMotorSettings(const usMotorSettings &other);

  //@}

private:
  //Settings from the 3D probe
  double m_motorRadius;
  double m_framePitch;
  usMotorType m_motorType;
};

#endif // US_MOTOR_SETTINGS_H
