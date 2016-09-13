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
 * @file usImagePreScan2D.h
 * @brief 2D prescan ultrasound image.
 * @author Pierre Chatelain
 */

//std include

//ViSP includes
#include <visp3/ustk_core/usImagePreScan2D.h>

//usTK includes

/**
* Basic constructor, all settings set to default.
*/
usImagePreScan2D::usImagePreScan2D() : vpImage<unsigned char>(), usImageSettings()
{

}

/**
* Initializing image size constructor.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
*/
usImagePreScan2D::usImagePreScan2D(unsigned int AN, unsigned int LN) : vpImage<unsigned char>(LN, AN), usImageSettings()
{

}

/**
* Initializing constructor for image size and probe settings.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements in the ultrasound probe.
*/
usImagePreScan2D::usImagePreScan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
				 float resolution, float BSampleFreq, float probeElementPitch) :
  vpImage<unsigned char>(AN, LN), usImageSettings(probeRadius, lineAngle, resolution, BSampleFreq, probeElementPitch)
{

}

/**
* Copy constructor.
* @param[in] other usImagePreScan2D image you want to copy.
*/
usImagePreScan2D::usImagePreScan2D(const usImagePreScan2D &other) :
  vpImage<unsigned char>(other), usImageSettings(other)
{

}

/**
* Destructor.
*/
usImagePreScan2D::~usImagePreScan2D() {};

/**
* Copy from vpImage.
* @param[in] I vpImage<unsigned char> to copy.
*/
void usImagePreScan2D::copyFrom(const vpImage<unsigned char> &I)
{
  resize(I.getHeight(), I.getWidth());
  memcpy(bitmap, I.bitmap, I.getSize()*sizeof(unsigned char));
}

/**
* Get the number of A-samples in a line.
* @param[out] AN number of A-samples in a line.
*/
unsigned int usImagePreScan2D::getAN() const { return getHeight(); }

/**
* Get the number of lines.
* @param[out] LN number of lines.
*/
unsigned int usImagePreScan2D::getLN() const { return getWidth(); }
