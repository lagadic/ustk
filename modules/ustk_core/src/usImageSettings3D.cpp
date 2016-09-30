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
 * @file usImageSettings3D.cpp
 * @brief Generic ultrasound 3D image settings.
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usImageSettings3D.h>

//ustk includes

/**
* Basic Constructor, all settings set to default.
*/
usImageSettings3D::usImageSettings3D() : usImageSettings(), m_motorRadius(0.0f), m_framePitch(0.0f) {}

/**
* Full Constructor, all settings availables
* @param probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param motorRadius Distance between the rotation center of the probe motor and the first pixel arc acquired, in meters (m).
* @param lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param frameAngle Radius between 2 successives acquisiton planes in the probe, in radians (rad).
* @param resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param BSampleFreq Sampling frequency used for B-Mode.
* @param probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements of the ultrasound probe.
*/
usImageSettings3D::usImageSettings3D(double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isImageConvex, bool isMotorConvex)
  : usImageSettings(probeRadius, scanLinePitch, isImageConvex), m_motorRadius(motorRadius), m_framePitch(framePitch), m_isMotorConvex(isMotorConvex) {}

/**
* Copy Constructor, all settings availables
* @param other usImageSettings3D you want to copy.
*/
usImageSettings3D::usImageSettings3D(const usImageSettings3D &other) : usImageSettings(other), m_motorRadius(other.getMotorRadius()), m_framePitch(other.getFramePitch()), m_isMotorConvex(other.isMotorConvex()) {}

/**
* Destructor.
*/
usImageSettings3D::~usImageSettings3D() {}

/**
* Assignment operator.
* @param other usImageSettings3D you want to copy.
*/
usImageSettings3D& usImageSettings3D::operator=(const usImageSettings3D& other)
{
  usImageSettings::operator =(other);
  m_motorRadius = other.getMotorRadius();
  m_framePitch = other.getFramePitch();
  m_isMotorConvex = other.isMotorConvex();
  return *this;
}

bool usImageSettings3D::operator==(const usImageSettings3D& other)
{
  return (usImageSettings::operator ==(other) &&
          this->getFramePitch() == other.getFramePitch() &&
          this->getMotorRadius() == other.getMotorRadius() &&
          this->isMotorConvex() == other.isMotorConvex());
}

//probe settings getters/setters

/**
* Set the motor  radius (m).
* @param motorRadius Motor radius in meters.
*/
void usImageSettings3D::setMotorRadius(double motorRadius) { m_motorRadius = motorRadius; }

/**
* Get the probe radius (m).
* @return motorRadius Motor radius in meters.
*/
double usImageSettings3D::getMotorRadius() const { return m_motorRadius; }

/**
* Set the frame angle (rad).
* @param angle Frame angle of the probe in radians.
*/
void usImageSettings3D::setFramePitch(double framePitch) { m_framePitch = framePitch; }

/**
* Get the frame angle (rad).
* @return m_lineAngle Frame angle of the probe in radians.
*/
double usImageSettings3D::getFramePitch() const { return m_framePitch; }

/**
* Set the motor type : convex or linear (from probe type used to acquire the image).
* @param isMotorConvex Boolean to specify the motor type : true for convex, false for linear.
*/
void usImageSettings3D::setMotorConvex(bool isMotorConvex) {
  m_isMotorConvex = isMotorConvex;
  if (!isMotorConvex) {
    setMotorRadius(0.0f);
  }
}

/**
* Get the motor type : convex or linear (from probe type used to acquire the image).
* @return isMotorConvex Boolean to specify the motor type : true for convex, false for linear.
*/
bool usImageSettings3D::isMotorConvex() const { return m_isMotorConvex; }

/**
* Print probe settings information.
*/
void usImageSettings3D::printProbeSettings() 
{
  usImageSettings::printProbeSettings();
  std::cout << "motor radius : " << m_motorRadius << std::endl
            << "frame angle : " << m_framePitch << std::endl;
}
