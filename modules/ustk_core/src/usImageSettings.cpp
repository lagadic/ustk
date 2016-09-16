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
 * @file usImageSettings.cpp
 * @brief Generic ultrasound image settings.
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usImageSettings.h>

//ustk includes

/**
* Basic Constructor, all settings set to default.
*/
usImageSettings::usImageSettings() : m_probeRadius(0.0f), m_lineAngle(0.0f),
                                        m_resolution(0.0f), m_BSampleFreq(0.0f),
                                        m_probeElementPitch(0.0f) {}
		   
/**
* Full Constructor, all settings availables
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements of the ultrasound probe.
*/
usImageSettings::usImageSettings(float probeRadius, float lineAngle,
                                 float resolution, float BSampleFreq, 
                                 float probeElementPitch) :     m_probeRadius(probeRadius), m_lineAngle(lineAngle),
                                                                m_resolution(resolution), m_BSampleFreq(BSampleFreq),
                                                                m_probeElementPitch(probeElementPitch) {}

/**
* Copy Constructor, all settings availables
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements of the ultrasound probe.
*/
usImageSettings::usImageSettings(const usImageSettings &other) {
	m_probeRadius = other.getProbeRadius();
	m_lineAngle = other.getLineAngle();
	m_resolution = other.getResolution();
	m_BSampleFreq = other.getBSampleFreq();
	m_probeElementPitch = other.getProbeElementPitch(); 
}
/**
* Destructor.
*/
usImageSettings::~usImageSettings() {}

/**
* Assignment operator.
* @param[in] other usImageSettings you want to copy.
*/
usImageSettings& usImageSettings::operator=(const usImageSettings& other)
{
  m_probeRadius = other.getProbeRadius();
  m_lineAngle = other.getLineAngle();
  m_resolution = other.getResolution();
  m_BSampleFreq = other.getBSampleFreq();
  m_probeElementPitch = other.getProbeElementPitch();

  return *this;
}

//probe settings getters/setters

/**
* Set the probe  radius (m).
* @param[in] probeRadius Probe radius in meters.
*/
void usImageSettings::setProbeRadius(float probeRadius) { m_probeRadius = probeRadius; }

/**
* Get the probe radius (m).
* @return m_probeRadius Probe radius in meters.
*/
float usImageSettings::getProbeRadius() const { return m_probeRadius; }

/**
* Set the line angle (rad).
* @param[in] angle Line angle of the probe in radians.
*/
void usImageSettings::setLineAngle(float angle) { m_lineAngle = angle; }

/**
* Get the line angle (rad).
* @return m_lineAngle Line angle of the probe in radians.
*/
float usImageSettings::getLineAngle() const { return m_lineAngle; }

/**
* Get the linear resolution = pixel size (m).
* @param[in] resolution Image resolution in meters.
*/
void usImageSettings::setResolution(float resolution) { m_resolution = resolution; }

/**
* Get the linear resolution = pixel size (m).
* @return m_resolution Image resolution in meters.
*/
float usImageSettings::getResolution() const { return m_resolution; }

/**
* Get the B-sample frequence (Hz).
* @param[in] freq B-mode sampling frequency in Hertz.
*/
void usImageSettings::setBSampleFreq(float freq) { m_BSampleFreq = freq; }

/**
* Get the B-sample frequence (Hz).
* @return m_BSampleFreq B-mode sampling frequency in Hertz.
*/
float usImageSettings::getBSampleFreq() const { return m_BSampleFreq; }

/**
* Set the probe element pitch.
* @param[in] probeElementPitch Probe element pitch in meters.
*/
void usImageSettings::setProbeElementPitch(float probeElementPitch) { m_probeElementPitch = probeElementPitch; }

/**
* Get the probe element pitch.
* @return m_probeElementPitch Probe element pitch in meters.
*/
float usImageSettings::getProbeElementPitch() const { return m_probeElementPitch; }


/**
* Print probe settings information.
*/
void usImageSettings::printProbeSettings()
{
	std::cout << "probe radius: " << m_probeRadius << std::endl
		<< "line angle: " << m_lineAngle << std::endl
		<< "resolution: " << m_resolution << std::endl
		<< "B sample frequency : " << m_BSampleFreq << std::endl
		<< "probe element pitch: " << m_probeElementPitch << std::endl;
}
