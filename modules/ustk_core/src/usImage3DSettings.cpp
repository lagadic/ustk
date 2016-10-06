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
 * @file usImage3DSettings.cpp
 * @brief Generic ultrasound 3D image settings.
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usImage3DSettings.h>

//ustk includes

/**
* Basic Constructor, all settings set to default.
*/
usImage3DSettings::usImage3DSettings() : usImageSettings(), m_motorRadius(0.0f), m_framePitch(0.0f) {}

/**
* Full Constructor, all settings availables
* @param probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param motorRadius Distance between the rotation center of the probe motor and the first pixel arc acquired, in meters (m).
* @param scanLinePitch Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param framePitch Radius between 2 successives acquisiton planes in the probe, in radians (rad).
* @param isTransducerConvex True if the probe transducer is convex, false otherwise.
* @param isMotorRotating True if the probe motor is rotating, false otherwise.
*/
usImage3DSettings::usImage3DSettings(double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isTransducerConvex, bool isMotorRotating)
  : usImageSettings(probeRadius, scanLinePitch, isTransducerConvex), m_motorRadius(motorRadius), m_framePitch(framePitch), m_isMotorRotating(isMotorRotating) {}

/**
* Copy Constructor, all settings availables
* @param other usImage3DSettings you want to copy.
*/
usImage3DSettings::usImage3DSettings(const usImage3DSettings &other) : usImageSettings(other), m_motorRadius(other.getMotorRadius()), m_framePitch(other.getFramePitch()), m_isMotorRotating(other.isMotorRotating()) {}

/**
* Destructor.
*/
usImage3DSettings::~usImage3DSettings() {}

/**
* Assignment operator.
* @param other usImage3DSettings you want to copy.
*/
usImage3DSettings& usImage3DSettings::operator=(const usImage3DSettings& other)
{
  usImageSettings::operator =(other);
  m_motorRadius = other.getMotorRadius();
  m_framePitch = other.getFramePitch();
  m_isMotorRotating = other.isMotorRotating();
  return *this;
}

bool usImage3DSettings::operator==(const usImage3DSettings& other)
{
  return (usImageSettings::operator ==(other) &&
          this->getFramePitch() == other.getFramePitch() &&
          this->getMotorRadius() == other.getMotorRadius() &&
          this->isMotorRotating() == other.isMotorRotating());
}

//probe settings getters/setters

/**
* Set the motor  radius (m).
* @param motorRadius Motor radius in meters.
*/
void usImage3DSettings::setMotorRadius(double motorRadius) { m_motorRadius = motorRadius; }

/**
* Get the probe radius (m).
* @return motorRadius Motor radius in meters.
*/
double usImage3DSettings::getMotorRadius() const { return m_motorRadius; }

/**
* Set the frame angle (rad).
* @param framePitch Frame angle of the probe in radians.
*/
void usImage3DSettings::setFramePitch(double framePitch) { m_framePitch = framePitch; }

/**
* Get the frame angle (rad).
* @return m_lineAngle Frame angle of the probe in radians.
*/
double usImage3DSettings::getFramePitch() const { return m_framePitch; }

/**
* Set the motor type : convex or linear (from probe type used to acquire the image).
* @param isMotorRotating Boolean to specify the motor type : true for a rotating motor, false for a linear motor.
*/
void usImage3DSettings::setMotorConvexity(bool isMotorRotating) {
  m_isMotorRotating = isMotorRotating;
  if (!isMotorRotating) {
    setMotorRadius(0.0f);
  }
}

/**
* Get the motor type : convex or linear (from probe type used to acquire the image).
* @return isMotorRotating Boolean to specify the motor type : true for convex, false for linear.
*/
bool usImage3DSettings::isMotorRotating() const { return m_isMotorRotating; }

/**
* Print probe settings information.
*/
void usImage3DSettings::printProbeSettings()
{
  usImageSettings::printProbeSettings();
  std::cout << "motor radius : " << m_motorRadius << std::endl
            << "frame angle : " << m_framePitch << std::endl;
}
