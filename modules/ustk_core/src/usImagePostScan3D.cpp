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
