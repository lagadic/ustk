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

#include <visp3/ustk_core/usImageSettings.h>

#include <iostream>

usImageSettings::usImageSettings() : m_scannerType(UNKNOWN_SCANNER), m_probeType(UNKNOWN_PROBE),
		   m_dataIdx(0), m_timestamp(0.0), m_originX(0), m_originY(0) {}
		   
usImageSettings::usImageSettings(float probeRadius, float lineAngle, float resolution, float BSampleFreq, float probeElementPitch) {
	m_probeRadius = probeRadius;
	m_lineAngle = lineAngle;
	m_resolution = resolution;
	m_BSampleFreq = BSampleFreq;
	m_probeElementPitch = probeElementPitch;
}

usImageSettings::~usImageSettings() {}

usImageSettings& usImageSettings::operator=(const usImageSettings& other)
{
  m_scannerType = other.m_scannerType;
  m_probeType = other.m_probeType;
  m_dataIdx = other.m_dataIdx;
  m_timestamp = other.m_timestamp;
  m_originX = other.m_originX;
  m_originY = other.m_originY;

  return *this;
}

usScannerType usImageSettings::getScannerType() const { return m_scannerType; }

usProbeType usImageSettings::getProbeType() const { return m_probeType; }

unsigned int usImageSettings::getDataIdx() const { return m_dataIdx; }

double usImageSettings::getTimestamp() const { return m_timestamp; }

double usImageSettings::getOriginX() const { return m_originX; }
double usImageSettings::getOriginY() const { return m_originY; }


void usImageSettings::setScannerType(usScannerType scannerType) { m_scannerType = scannerType; }

void usImageSettings::setProbeType(usProbeType probeType) { m_probeType = probeType; }

void usImageSettings::setDataIdx(unsigned int idx) { m_dataIdx = idx; }

void usImageSettings::setTimestamp(double timestamp) { m_timestamp = timestamp; }

void usImageSettings::setOriginX(double originX) { m_originX = originX; }
void usImageSettings::setOriginY(double originY) { m_originY = originY; }

void usImageSettings::setOrigin(double originX, double originY) {
  m_originX = originX;
  m_originY = originY;
}

void usImageSettings::printInfo()
{
  std::cout << "scanner: " << m_scannerType << std::endl
	    << "probe: " << m_probeType << std::endl
	    << "data index: " << m_dataIdx << std::endl
	    << "timestamp: " << m_timestamp << std::endl
	    << "origin: (" << m_originX << ", " << m_originY << ")" << std::endl;
}

//probe settings getters/setters

float usImageSettings::getProbeRadius() const { return m_probeRadius; }
float usImageSettings::getLineAngle() const { return m_lineAngle; }
float usImageSettings::getResolution() const { return m_resolution; }
float usImageSettings::getBSampleFreq() const { return m_BSampleFreq; }
float usImageSettings::getProbeElementPitch() const { return m_probeElementPitch; }

void usImageSettings::setProbeRadius(float probeRadius) { m_probeRadius = probeRadius; }
void usImageSettings::setLineAngle(double angle) { m_lineAngle = angle; }
void usImageSettings::setResolution(float resolution) { m_resolution = resolution; }
void usImageSettings::setBSampleFreq(double freq) { m_BSampleFreq = freq; }
void usImageSettings::setProbeElementPitch(float probeElementPitch) { m_probeElementPitch = probeElementPitch; }

void usImageSettings::printProbeSettings()
{
	std::cout << "probe radius: " << m_probeRadius << std::endl
		<< "line angle: " << m_lineAngle << std::endl
		<< "resolution: " << m_resolution << std::endl
		<< "B sample frequency : " << m_BSampleFreq << std::endl
		<< "probe element pitch: " << m_probeElementPitch << std::endl
		<< "probe element pitch: " << probeSettingsAreValid << std::endl;
}
