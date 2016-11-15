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
* Marc Pouliquen
*
*****************************************************************************/

/**
* @file usMeterPixelConversion.h
* @brief Conversion between a position in meters in the space and the equivalent pixel position in the ultrasound image.
*/

#ifndef US_METER_PIXEL_CONVERSION_H
#define US_METER_PIXEL_CONVERSION_H

#include <algorithm>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>

/**
* @class usMeterPixelConversion
* @brief Conversion between a position in meters in the space and the equivalent pixel position in the ultrasound image.
* @ingroup module_ustk_core
*/
class VISP_EXPORT usMeterPixelConversion
{
public:
  //Only post-scan images can manage this kind of conversion
  static void convert(const usImagePostScan2D<unsigned char> &image, const double &x, const double &y,
                                                                    double &u,  double &v);

  static void convert(const usImagePostScan3D<unsigned char> &image, const double &x, const double &y, const double &z,
                                                                    double &u,  double &v, double &w);
};

#endif //US_METER_PIXEL_CONVERSION_H
