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
* @file usImagePostScan3D.cpp
* @brief 3D prescan ultrasound image.
*/

//std includes

//ViSP includes

//ustk includes
#include <visp3/ustk_core/usImagePostScan3D.h>

/**
* Basic constructor, all parameters set to default values
*/
usImagePostScan3D::usImagePostScan3D() : usImage3D<unsigned char>(), usImageSettings3D()
{

}

/**
* Complete constructor, all parameters availables.
* @param AN A-samples in a line (corresponds to image height in px).
* @param LN Number of lines (corresponds to image width in px).
* @param FN Number of Frames.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param isImageConvex Boolean to specyfy if the image was acquired by a convex probe(true) or by a linear probe (false).
* @param isMotorConvex Boolean to specyfy if the image was acquired by a rotating  motor(true) or by a linear motor (false).
*/
usImagePostScan3D::usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isImageConvex, bool isMotorConvex)
  : usImage3D<unsigned char>(), usImageSettings3D(probeRadius, motorRadius, scanLinePitch, framePitch, isImageConvex, isMotorConvex)
{

}

/**
* Copy constructor from other usImagePostScan3D
* @param other usImagePostScan3D to copy
*/
usImagePostScan3D::usImagePostScan3D(const usImagePostScan3D &other) : usImage3D<unsigned char>(other), usImageSettings3D(other)
{

}


/**
* Constructor from usImage3D
* @param other usImage3D<unsigned char> to copy
*/
usImagePostScan3D::usImagePostScan3D(const usImage3D<unsigned char> &other) : usImage3D<unsigned char>(other)
{

}

/**
* Constructor from usImageSettings3D.
* @param other usImageSettings3D to copy
*/
usImagePostScan3D::usImagePostScan3D(const usImageSettings3D &other) : usImageSettings3D(other)
{

}

/**
* Constructor from usImage3D and usImageSettings3D.
* @param otherImage usImage3D<unsigned char> to copy
* @param otherSettings usImageSettings3D to copy
*/
usImagePostScan3D::usImagePostScan3D(const usImage3D<unsigned char> &otherImage, const usImageSettings3D &otherSettings) : usImage3D<unsigned char>(otherImage), usImageSettings3D(otherSettings)
{

}

/**
* Destructor.
*/
usImagePostScan3D::~usImagePostScan3D() {}

/**
* Setter for width Resolution.
* @param widthResolution Width resolution (in meters) to set.
*/
void usImagePostScan3D::setWidthResolution(double widthResolution) { m_widthResolution = widthResolution; }

/**
* Getter for width Resolution.
* @return widthResolution Width resolution (in meters).
*/
double usImagePostScan3D::getWidthResolution() { return m_heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set.
*/
void usImagePostScan3D::setHeightResolution(double heightResolution) { m_heightResolution = heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set.
*/
double usImagePostScan3D::getHeightResolution() { return m_heightResolution; }