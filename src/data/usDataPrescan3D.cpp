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

#include "usDataPrescan3D.h"
#include <cstring>

usDataPrescan3D::usDataPrescan3D() : usVolume<unsigned char>(),
				     m_probeRadius(0.0f), m_motorRadius(0.0f),
				     m_lineAngle(0.0f), m_frameAngle(0.0f), m_resolution(0.0f),
				     m_BSampleFreq(0.0f), m_probeElementPitch(0.0f)
{
  setMode(PRESCAN_3D);
}

usDataPrescan3D::usDataPrescan3D(unsigned int AN, unsigned int LN, unsigned int FN)
  : usVolume<unsigned char>(AN, LN, FN, 1.0, 1.0, 1.0)
{
  m_probeRadius = 0.0f;
  m_motorRadius = 0.0f;
  m_lineAngle = 0.0f;
  m_frameAngle = 0.0f;
  m_resolution = 0.0f;
  m_BSampleFreq = 0.0f;
  m_probeElementPitch = 0.0f;
  setMode(PRESCAN_3D);
}

usDataPrescan3D::usDataPrescan3D(unsigned int AN, unsigned int LN, unsigned int FN,
				 float probeRadius, float motorRadius,
				 float lineAngle, float frameAngle, float resolution,
				 float BSampleFreq, float probeElementPitch)
  : usVolume<unsigned char>(AN, LN, FN, 1.0, 1.0, 1.0)
{
  m_probeRadius = probeRadius;
  m_motorRadius = motorRadius;
  m_lineAngle = lineAngle;
  m_frameAngle = frameAngle;
  m_resolution = resolution;
  m_BSampleFreq = BSampleFreq;
  m_probeElementPitch = probeElementPitch;
  setMode(PRESCAN_3D);
}

usDataPrescan3D::usDataPrescan3D(const usDataPrescan3D& other, const bool copy)
  : usVolume<unsigned char>(other, copy), usData(other)
{
  m_probeRadius = other.m_probeRadius;
  m_motorRadius = other.m_motorRadius;
  m_lineAngle = other.m_lineAngle;
  m_frameAngle = other.m_frameAngle;
  m_resolution = other.m_resolution;
  m_BSampleFreq = other.m_BSampleFreq;
  m_probeElementPitch = other.m_probeElementPitch;
  setMode(PRESCAN_3D);
}

usDataPrescan3D& usDataPrescan3D::operator=(const usDataPrescan3D& other)
{
  usData::operator=(other);
  usVolume<unsigned char>::operator=(other);
  m_probeRadius = other.m_probeRadius;
  m_motorRadius = other.m_motorRadius;
  m_lineAngle = other.m_lineAngle;
  m_frameAngle = other.m_frameAngle;
  m_resolution = other.m_resolution;
  m_BSampleFreq = other.m_BSampleFreq;
  m_probeElementPitch = other.m_probeElementPitch;
  setMode(PRESCAN_3D);
  return *this;
}
