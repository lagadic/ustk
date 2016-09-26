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
 * @file usImagePostScan2D.cpp
 * @brief 2D prescan ultrasound image.
 */

//std includes

//ViSP includes

//ustk includes
#include <visp3/ustk_core/usImagePostScan2D.h>

/**
* Basic constructor, all parameters set to default values
*/
usImagePostScan2D::usImagePostScan2D() : vpImage<unsigned char>(), usImageSettings()
{
	
}

/**
* Full constructor, all parameters settables.
* @param AN number of A-samples in a line.
* @param LN number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle(rad) / Distance(m) between 2 lines of the ultrasound probe used to acquire the RF image. Angle if isConvex is true, distance if it's false.
* @param isConvex Boolean to specify if the probe used was convex(true) or linear(false).
*/
usImagePostScan2D::usImagePostScan2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex)
  : vpImage<unsigned char>(AN,LN), usImageSettings(probeRadius, scanLinePitch, isConvex)
{

}

/**
* Copy constructor from other usImagePostScan2D
* @param other usImagePostScan2D to copy
*/
usImagePostScan2D::usImagePostScan2D(const usImagePostScan2D &other) : vpImage<unsigned char>(other), usImageSettings(other)
{

}


/**
* Constructor from vpImage
* @param other vpImage<unsigned char> to copy
*/
usImagePostScan2D::usImagePostScan2D(const vpImage<unsigned char> &other) : vpImage<unsigned char>(other)
{

}

/**
* Constructor from usImageSettings.
* @param other usImageSettings to copy
*/
usImagePostScan2D::usImagePostScan2D(const usImageSettings &other) : usImageSettings(other)
{

}

/**
* Constructor from vpImage and usImageSettings.
* @param otherImage vpImage<unsigned char> to copy
* @param otherSettings usImageSettings to copy
*/
usImagePostScan2D::usImagePostScan2D(const vpImage<unsigned char> &otherImage, const usImageSettings &otherSettings) : vpImage<unsigned char>(otherImage), usImageSettings(otherSettings)
{

}

/**
* Destructor.
*/
usImagePostScan2D::~usImagePostScan2D() {}

/**
* Setter for width Resolution.
* @param widthResolution Width resolution (in meters) to set.
*/
void usImagePostScan2D::setWidthResolution(double widthResolution) { m_widthResolution = widthResolution; }

/**
* Getter for width Resolution.
* @return widthResolution Width resolution (in meters).
*/
double usImagePostScan2D::getWidthResolution() { return m_heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set.
*/
void usImagePostScan2D::setHeightResolution(double heightResolution) { m_heightResolution = heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set. 
*/
double usImagePostScan2D::getHeightResolution() { return m_heightResolution; }