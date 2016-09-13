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
 * @brief Generic ultrasound data.
 * @author Pierre Chatelain
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usImageSettings.h>

//ustk includes

/**
* Basic Constructor, all settings set to default.
*/
usImageSettings::usImageSettings() : m_scannerType(UNKNOWN_SCANNER), m_probeType(UNKNOWN_PROBE),
		   m_dataIdx(0), m_timestamp(0.0), m_originX(0), m_originY(0) {}
		   
/**
* Full Constructor, all settings availables
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements of the ultrasound probe.
*/
usImageSettings::usImageSettings(float probeRadius, float lineAngle, float resolution, float BSampleFreq, float probeElementPitch) {
	m_probeRadius = probeRadius;
	m_lineAngle = lineAngle;
	m_resolution = resolution;
	m_BSampleFreq = BSampleFreq;
	m_probeElementPitch = probeElementPitch;
}

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
	m_scannerType = other.getScannerType();
	m_probeType = other.getProbeType();
	m_dataIdx = other.getDataIdx();
	m_timestamp = other.getTimestamp();
	m_originX = other.getOriginX();
	m_originY = other.getOriginY();
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
  m_scannerType = other.getScannerType();
  m_probeType = other.getProbeType();
  m_dataIdx = other.getDataIdx();
  m_timestamp = other.getTimestamp();
  m_originX = other.getOriginX();
  m_originY = other.getOriginY();
  m_probeRadius = other.getProbeRadius();
  m_lineAngle = other.getLineAngle();
  m_resolution = other.getResolution();
  m_BSampleFreq = other.getBSampleFreq();
  m_probeElementPitch = other.getProbeElementPitch();

  return *this;
}

/**
* Set the scanner type (SONIX_RP, SONIX_TOUCH, SONOSITE).
* @param[in] scannerType Scanner used to acquire the image.
*/
void usImageSettings::setScannerType(usScannerType scannerType) { m_scannerType = scannerType; }

/**
* Get the scanner type (SONIX_RP, SONIX_TOUCH, SONOSITE).
* @param[out] scannerType Scanner used to acquire the image.
*/
usScannerType usImageSettings::getScannerType() const { return m_scannerType; }

/**
* Set the probe type (US_4DC7, SS_C60).
* @param[in] probeType Scanner used to acquire the image.
*/
void usImageSettings::setProbeType(usProbeType probeType) { m_probeType = probeType; }

/**
* Get the probe type (US_4DC7, SS_C60).
* @param[out] m_probeType Scanner used to acquire the image.
*/
usProbeType usImageSettings::getProbeType() const { return m_probeType; }

/**
* Set the data index.
* @param[in] idx index of the image.
*/
void usImageSettings::setDataIdx(unsigned int idx) { m_dataIdx = idx; }

/**
* Get the data index.
* @param[out] m_dataIdx index of the image.
*/
unsigned int usImageSettings::getDataIdx() const { return m_dataIdx; }

/**
* Set the data timestamp.
* @param[in] timestamp Timestamp of the image.
*/
void usImageSettings::setTimestamp(double timestamp) { m_timestamp = timestamp; }

/**
* Set the data timestamp.
* @param[out] m_timestamp Timestamp of the image.
*/
double usImageSettings::getTimestamp() const { return m_timestamp; }

/**
* Set the x-coordinate of the volume origin.
* @param[in] originX x-coordinate of the image origin.
*/
void usImageSettings::setOriginX(double originX) { m_originX = originX; }

/**
* Get the x-coordinate of the volume origin.
* @param[out] m_originX x-coordinate of the image origin.
*/
double usImageSettings::getOriginX() const { return m_originX; }

/**
* Set the y-coordinate of the volume origin.
* @param[in] originY y-coordinate of the image origin.
*/
void usImageSettings::setOriginY(double originY) { m_originY = originY; }

/**
* Get the y-coordinate of the volume origin.
* @param[out] m_originY y-coordinate of the image origin.
*/
double usImageSettings::getOriginY() const { return m_originY; }

/**
* Set the coordinates of the image origin.
* @param[in] originX x-coordinate of the image origin.
* @param[in] originY y-coordinate of the image origin.
*/
void usImageSettings::setOrigin(double originX, double originY) {
  m_originX = originX;
  m_originY = originY;
}

/**
* Print data information.
*/
void usImageSettings::printInfo()
{
  std::cout << "scanner: " << m_scannerType << std::endl
	    << "probe: " << m_probeType << std::endl
	    << "data index: " << m_dataIdx << std::endl
	    << "timestamp: " << m_timestamp << std::endl
	    << "origin: (" << m_originX << ", " << m_originY << ")" << std::endl;
}

//probe settings getters/setters

/**
* Set the probe  radius (m).
* @param[in] probeRadius Probe radius in meters.
*/
void usImageSettings::setProbeRadius(float probeRadius) { m_probeRadius = probeRadius; }

/**
* Get the probe radius (m).
* @param[out] m_probeRadius Probe radius in meters.
*/
float usImageSettings::getProbeRadius() const { return m_probeRadius; }

/**
* Set the line angle (rad).
* @param[in] angle Line angle of the probe in radians.
*/
void usImageSettings::setLineAngle(float angle) { m_lineAngle = angle; }

/**
* Get the line angle (rad).
* @param[out] m_lineAngle Line angle of the probe in radians.
*/
float usImageSettings::getLineAngle() const { return m_lineAngle; }

/**
* Get the linear resolution = pixel size (m).
* @param[in] resolution Image resolution in meters.
*/
void usImageSettings::setResolution(float resolution) { m_resolution = resolution; }

/**
* Get the linear resolution = pixel size (m).
* @param[out] m_resolution Image resolution in meters.
*/
float usImageSettings::getResolution() const { return m_resolution; }

/**
* Get the B-sample frequence (Hz).
* @param[in] freq B-mode sampling frequency in Hertz.
*/
void usImageSettings::setBSampleFreq(float freq) { m_BSampleFreq = freq; }

/**
* Get the B-sample frequence (Hz).
* @param[out] m_BSampleFreq B-mode sampling frequency in Hertz.
*/
float usImageSettings::getBSampleFreq() const { return m_BSampleFreq; }

/**
* Set the probe element pitch.
* @param[in] probeElementPitch Probe element pitch in meters.
*/
void usImageSettings::setProbeElementPitch(float probeElementPitch) { m_probeElementPitch = probeElementPitch; }

/**
* Get the probe element pitch.
* @param[out] m_probeElementPitch Probe element pitch in meters.
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
