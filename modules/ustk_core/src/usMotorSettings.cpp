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
 * @file usMotorSettings.cpp
 * @brief Generic ultrasound 3D image settings.
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usMotorSettings.h>

//ustk includes

/**
* Basic Constructor, all settings set to default.
*/
usMotorSettings::usMotorSettings() : m_motorRadius(0.0), m_framePitch(0.0), m_motorType(LinearMotor) {}

/**
* Full Constructor, all settings availables
* @param motorRadius Distance between the rotation center of the probe motor and the first pixel arc acquired, in meters (m).
* @param framePitch Pitch between two sucessive frames . In meters if motorType is linear, in radians (rad) otherwise.
* @param motorType Motor type of the probe used.
*/
usMotorSettings::usMotorSettings(double motorRadius, double framePitch, usMotorType motorType)
: m_motorRadius(motorRadius), m_framePitch(framePitch), m_motorType(motorType)
{

}

/**
* Copy Constructor, all settings availables
* @param other usMotorSettings you want to copy.
*/
usMotorSettings::usMotorSettings(const usMotorSettings &other) : m_motorRadius(other.getMotorRadius()),
 m_framePitch(other.getFramePitch()), m_motorType(other.getMotorType())
 {

 }

/**
* Destructor.
*/
usMotorSettings::~usMotorSettings() {}

/**
* Assignment operator.
* @param other usMotorSettings you want to copy.
*/
usMotorSettings& usMotorSettings::operator=(const usMotorSettings& other)
{
  m_motorRadius = other.getMotorRadius();
  m_framePitch = other.getFramePitch();
  m_motorType = other.getMotorType();
  return *this;
}

bool usMotorSettings::operator==(const usMotorSettings& other)
{
  return (this->getFramePitch() == other.getFramePitch() &&
          this->getMotorRadius() == other.getMotorRadius() &&
          this->getMotorType() == other.getMotorType());
}

/**
* Print probe settings information.
*/
VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usMotorSettings& other)
{
  return out << "motor radius : " << other.getMotorRadius() << std::endl
    << "frame angle : " << other.getFramePitch() << std::endl
    << "motor type : " << other.getMotorType() << std::endl;
}

//probe settings getters/setters

/**
* Set the motor  radius (m).
* @param motorRadius Motor radius in meters.
*/
void usMotorSettings::setMotorRadius(double motorRadius)
{
  m_motorRadius = motorRadius;
}

/**
* Get the motor radius (m).
* @return motorRadius Motor radius in meters.
*/
double usMotorSettings::getMotorRadius() const
{
  return m_motorRadius;
}

/**
* Set the frame angle (rad).
* @param framePitch Frame angle of the probe in radians.
*/
void usMotorSettings::setFramePitch(double framePitch)
{
  m_framePitch = framePitch;
}

/**
* Get the frame pitch (radians or meters).
* @return framePitch Frame pitch between two sucessive frames. In meters if motorType is linear, in radians otherwise.
*/
double usMotorSettings::getFramePitch() const
{
  return m_framePitch;
}

/**
* Set the motor type : convex or linear (from probe type used to acquire the image).
* @param motorType Motor type to specify the motor type : LinearMotor, TiltingMotor (for a rotative motor), RotationalMotor (for a 360&deg; rotative motor).
*/
void usMotorSettings::setMotorType(usMotorType motorType)
{
  m_motorType = motorType;
  if (motorType== LinearMotor) {
    setMotorRadius(0.0);
  }
}

/**
* Get the motor type : linear, titling (small rotation angle) or rotational (360&deg; rotation).
* @return usMotorType to get the motor type.
*/
usMotorSettings::usMotorType usMotorSettings::getMotorType() const
{
  return m_motorType;
}

/**
* Set the motor settings from other usMotorSettings.
* @return usMotorSettings instance modified.
*/
void usMotorSettings::setMotorSettings(const usMotorSettings &other)
{
  *this = other;
}

