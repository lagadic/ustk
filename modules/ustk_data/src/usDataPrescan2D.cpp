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

#include <visp3/ustk_data/usDataPrescan2D.h>

usDataPrescan2D::usDataPrescan2D()
{
  m_probeRadius = 0.0f;
  m_lineAngle = 0.0f;
  m_resolution = 0.0f;
  m_BSampleFreq = 0.0f;
  m_probeElementPitch = 0.0f;
  setMode(PRESCAN_2D);
}

usDataPrescan2D::usDataPrescan2D(unsigned int AN, unsigned int LN) : vpImage<unsigned char>(LN, AN)
{
  m_probeRadius = 0.0f;
  m_lineAngle = 0.0f;
  m_resolution = 0.0f;
  m_BSampleFreq = 0.0f;
  m_probeElementPitch = 0.0f;
  setMode(PRESCAN_2D);
}

usDataPrescan2D::usDataPrescan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
				 float resolution, float BSampleFreq, float probeElementPitch) :
  vpImage<unsigned char>(AN, LN)
{
  m_probeRadius = probeRadius;
  m_lineAngle = lineAngle;
  m_resolution = resolution;
  m_BSampleFreq = BSampleFreq;
  m_probeElementPitch = probeElementPitch;
  setMode(PRESCAN_2D);
}

usDataPrescan2D::usDataPrescan2D(const usDataPrescan2D &other, const bool copy) :
  vpImage<unsigned char>(other), usData(other)
{
  m_probeRadius = other.getProbeRadius();
  m_lineAngle = other.getLineAngle();
  m_resolution = other.getResolution();
  m_BSampleFreq = other.getBSampleFreq();
  m_probeElementPitch = other.getProbeElementPitch();
  setMode(PRESCAN_2D);
}

usDataPrescan2D::~usDataPrescan2D() {};

void usDataPrescan2D::copyFrom(const vpImage<unsigned char> &I)
{
  resize(I.getHeight(), I.getWidth());
  memcpy(bitmap, I.bitmap, I.getSize()*sizeof(unsigned char));
}

unsigned int usDataPrescan2D::getAN() const { return getHeight(); }
unsigned int usDataPrescan2D::getLN() const { return getWidth(); }
float usDataPrescan2D::getProbeRadius() const { return m_probeRadius; }
float usDataPrescan2D::getLineAngle() const { return m_lineAngle; }
float usDataPrescan2D::getResolution() const { return m_resolution; }
float usDataPrescan2D::getBSampleFreq() const { return m_BSampleFreq; }
float usDataPrescan2D::getProbeElementPitch() const { return m_probeElementPitch; }

void usDataPrescan2D::setProbeRadius(float probeRadius) { m_probeRadius = probeRadius; }
void usDataPrescan2D::setLineAngle(double angle) { m_lineAngle = angle; }
void usDataPrescan2D::setResolution(float resolution) { m_resolution = resolution; }
void usDataPrescan2D::setBSampleFreq(double freq) { m_BSampleFreq = freq; }
void usDataPrescan2D::setProbeElementPitch(float probeElementPitch) { m_probeElementPitch = probeElementPitch; }
