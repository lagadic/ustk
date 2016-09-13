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

#include <visp3/ustk_core/usImagePreScan2D.h>

usImagePreScan2D::usImagePreScan2D()
{
  setProbeRadius(0.0f);
  setLineAngle(0.0f);
  setResolution(0.0f);
  setBSampleFreq(0.0f);
  setProbeElementPitch(0.0f);
}

usImagePreScan2D::usImagePreScan2D(unsigned int AN, unsigned int LN) : vpImage<unsigned char>(LN, AN)
{
  setProbeRadius(0.0f);
  setLineAngle(0.0f);
  setResolution(0.0f);
  setBSampleFreq(0.0f);
  setProbeElementPitch(0.0f);
}

usImagePreScan2D::usImagePreScan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
				 float resolution, float BSampleFreq, float probeElementPitch) :
  vpImage<unsigned char>(AN, LN)
{
  setProbeRadius(probeRadius);
  setLineAngle(lineAngle);
  setResolution(resolution);
  setBSampleFreq(BSampleFreq);
  setProbeElementPitch(probeElementPitch);
}

usImagePreScan2D::usImagePreScan2D(const usImagePreScan2D &other, const bool copy) :
  vpImage<unsigned char>(other), usImageSettings(other)
{
  setProbeRadius(other.getProbeRadius());
  setLineAngle(other.getLineAngle());
  setResolution(other.getResolution());
  setBSampleFreq(other.getBSampleFreq());
  setProbeElementPitch(other.getProbeElementPitch());
}

usImagePreScan2D::~usImagePreScan2D() {};

void usImagePreScan2D::copyFrom(const vpImage<unsigned char> &I)
{
  resize(I.getHeight(), I.getWidth());
  memcpy(bitmap, I.bitmap, I.getSize()*sizeof(unsigned char));
}

unsigned int usImagePreScan2D::getAN() const { return getHeight(); }
unsigned int usImagePreScan2D::getLN() const { return getWidth(); }
