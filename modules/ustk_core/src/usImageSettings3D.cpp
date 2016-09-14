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
usImageSettings3D::usImageSettings3D() : usImageSettings(), m_motorRadius(0.0f), m_frameAngle(0.0f) {}
		   
/**
* Full Constructor, all settings availables
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] motorRadius Distance between the rotation center of the probe motor and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] frameAngle Radius between 2 successives acquisiton planes in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements of the ultrasound probe.
*/
usImageSettings3D::usImageSettings3D(float probeRadius, float motorRadius, float lineAngle, float frameAngle,
                                     float resolution, float BSampleFreq, 
                                     float probeElementPitch) : usImageSettings(probeRadius, lineAngle,
                                                                                resolution, BSampleFreq,
                                                                                probeElementPitch), m_motorRadius(motorRadius), m_frameAngle(frameAngle) {}

/**
* Copy Constructor, all settings availables
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] motorRadius Distance between the rotation center of the probe motor and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements of the ultrasound probe.
*/
usImageSettings3D::usImageSettings3D(const usImageSettings3D &other) {
	setProbeRadius(other.getProbeRadius());
  setLineAngle(other.getLineAngle());
  setResolution(other.getResolution());
  setBSampleFreq(other.getBSampleFreq());
  setProbeElementPitch(other.getProbeElementPitch());
  m_motorRadius = other.getMotorRadius();
  m_frameAngle = other.getFrameAngle();
}

/**
* Destructor.
*/
usImageSettings3D::~usImageSettings3D() {}

/**
* Assignment operator.
* @param[in] other usImageSettings3D you want to copy.
*/
usImageSettings3D& usImageSettings3D::operator=(const usImageSettings3D& other)
{
  setProbeRadius(other.getProbeRadius());
  setLineAngle(other.getLineAngle());
  setResolution(other.getResolution());
  setBSampleFreq(other.getBSampleFreq());
  setProbeElementPitch(other.getProbeElementPitch());
  m_motorRadius = other.getMotorRadius();
  m_frameAngle = other.getFrameAngle();

  return *this;
}

//probe settings getters/setters

/**
* Set the motor  radius (m).
* @param[in] motorRadius Motor radius in meters.
*/
void usImageSettings3D::setMotorRadius(float motorRadius) { m_motorRadius = motorRadius; }

/**
* Get the probe radius (m).
* @return motorRadius Motor radius in meters.
*/
float usImageSettings3D::getMotorRadius() const { return m_motorRadius; }

/**
* Set the frame angle (rad).
* @param[in] angle Frame angle of the probe in radians.
*/
void usImageSettings3D::setFrameAngle(float angle) { m_frameAngle = angle; }

/**
* Get the frame angle (rad).
* @return m_lineAngle Frame angle of the probe in radians.
*/
float usImageSettings3D::getFrameAngle() const { return m_frameAngle; }

/**
* Print probe settings information.
*/
void usImageSettings3D::printProbeSettings() 
{
  usImageSettings::printProbeSettings();
  std::cout << "motor radius: " << m_motorRadius << std::endl
            << "frame angle: " << m_frameAngle << std::endl;
}
