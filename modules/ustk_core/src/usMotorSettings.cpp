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
usMotorSettings::usMotorSettings()
  : m_motorRadius(0.0), m_framePitch(0.0), m_motorType(LinearMotor)
{}

/**
* Full Constructor, all settings availables
* @param motorRadius Distance between the rotation center of the probe motor and the first pixel arc acquired, in meters (m).
* @param framePitch Pitch between two sucessive frames. In meters if motorType is linear, in radians (rad) otherwise.
* @param frameNumber Number of frame acquired by the probe.
* @param motorType Probe motor type.
*/
usMotorSettings::usMotorSettings(double motorRadius, double framePitch,
                                 unsigned int frameNumber, const usMotorType &motorType)
  : m_motorRadius(motorRadius), m_framePitch(framePitch), m_frameNumber(frameNumber), m_motorType(motorType)
{

}

/**
* Copy Constructor, all settings availables
* @param other usMotorSettings you want to copy.
*/
usMotorSettings::usMotorSettings(const usMotorSettings &other)
  : m_motorRadius(other.getMotorRadius()), m_framePitch(other.getFramePitch()),
    m_frameNumber(other.getFrameNumber()), m_motorType(other.getMotorType())
 {

 }

/**
* Destructor.
*/
usMotorSettings::~usMotorSettings() {}

/**
* Assignment operator.
* @param other Motor settings you want to copy.
*/
usMotorSettings& usMotorSettings::operator=(const usMotorSettings& other)
{
  m_motorRadius = other.getMotorRadius();
  m_framePitch = other.getFramePitch();
  m_frameNumber = other.getFrameNumber();
  m_motorType = other.getMotorType();
  return *this;
}

/**
 * Compare two motor settings.
 * @param other Motor settings to compare.
 * @return true if settings are the same, false otherwise.
 */
bool usMotorSettings::operator==(const usMotorSettings& other)
{
  return (this->getFramePitch() == other.getFramePitch() &&
          this->getFrameNumber() == other.getFrameNumber() &&
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
             << "frame number : " << other.getFrameNumber() << std::endl
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
* Getter for frame number.
* @return The number of frames used for 3D acquisition.
*/
unsigned int usMotorSettings::getFrameNumber() const
{
  return m_frameNumber;
}

/**
* Set the motor type : convex or linear (from probe type used to acquire the image).
* @param motorType Motor type to specify the motor type : LinearMotor, TiltingMotor (for a rotative motor), RotationalMotor (for a 360&deg; rotative motor).
*/
void usMotorSettings::setMotorType(const usMotorType &motorType)
{
  m_motorType = motorType;
  if (motorType == LinearMotor) {
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
* Set the motor settings from other motor settings.
* @return usMotorSettings Settings to copy.
*/
void usMotorSettings::setMotorSettings(const usMotorSettings &other)
{
  *this = other;
}

/**
* Setter for frame number.
* @param frameNumber The number of frames used for 3D acquisition.
*/
void usMotorSettings::setFrameNumber(unsigned int frameNumber)
{
  m_frameNumber = frameNumber;
}
