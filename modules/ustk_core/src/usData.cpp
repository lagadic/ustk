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

#include <visp3/ustk_data/usData.h>

#include <iostream>

usData::usData() : m_mode(UNKNOWN_MODE), m_scannerType(UNKNOWN_SCANNER), m_probeType(UNKNOWN_PROBE),
		   m_dataIdx(0), m_timestamp(0.0), m_originX(0), m_originY(0), m_originZ(0) {}

usData::~usData() {}

usData& usData::operator=(const usData& other)
{
  m_mode = other.m_mode;
  m_scannerType = other.m_scannerType;
  m_probeType = other.m_probeType;
  m_dataIdx = other.m_dataIdx;
  m_timestamp = other.m_timestamp;
  m_originX = other.m_originX;
  m_originY = other.m_originY;
  m_originZ = other.m_originZ;

  return *this;
}

usDataMode usData::getMode() const { return m_mode; }

usScannerType usData::getScannerType() const { return m_scannerType; }

usProbeType usData::getProbeType() const { return m_probeType; }

unsigned int usData::getDataIdx() const { return m_dataIdx; }

double usData::getTimestamp() const { return m_timestamp; }

double usData::getOriginX() const { return m_originX; }
double usData::getOriginY() const { return m_originY; }
double usData::getOriginZ() const { return m_originZ; }

void usData::setMode(usDataMode mode) { m_mode = mode; }

void usData::setScannerType(usScannerType scannerType) { m_scannerType = scannerType; }

void usData::setProbeType(usProbeType probeType) { m_probeType = probeType; }

void usData::setDataIdx(unsigned int idx) { m_dataIdx = idx; }

void usData::setTimestamp(double timestamp) { m_timestamp = timestamp; }

void usData::setOriginX(double originX) { m_originX = originX; }
void usData::setOriginY(double originY) { m_originY = originY; }
void usData::setOriginZ(double originZ) { m_originZ = originZ; }

void usData::setOrigin(double originX, double originY, double originZ) {
  m_originX = originX;
  m_originY = originY;
  m_originZ = originZ;
}

void usData::printInfo()
{
  std::cout << "mode: " << m_mode << std::endl
	    << "scanner: " << m_scannerType << std::endl
	    << "probe: " << m_probeType << std::endl
	    << "data index: " << m_dataIdx << std::endl
	    << "timestamp: " << m_timestamp << std::endl
	    << "origin: (" << m_originX << ", " << m_originY << ", " << m_originZ << ")" << std::endl;
}
