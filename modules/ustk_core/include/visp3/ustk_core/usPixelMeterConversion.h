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
* @file usPixelMeterConversion.h
* @brief Conversion between a pixel position in the ultrasound image and the real position in meters.
*/

#ifndef US_PIXEL_METER_CONVERSION_H
#define US_PIXEL_METER_CONVERSION_H

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>

/**
* @class usPixelMeterConversion
* @brief Conversion between a pixel position in the ultrasound image and the real position in meters.
* @ingroup module_ustk_core
*
* @warning Be sure you correctly filled your acquisition settings (probe raduis, scanlinePitch, etc...). Those parameters are used in the conversion !
*/
class VISP_EXPORT usPixelMeterConversion
{
public:
  //Only post-scan images can manage this kind of conversion
  static void convert(const usImagePostScan2D<unsigned char> &image, const double &u, const double &v,
                                                                    double &x,  double &y);

  static void convert(const usImagePostScan3D<unsigned char> &image, const double &u, const double &v, const double &w,
                                                                    double &x,  double &y, double &z);
};

#endif //US_PIXEL_METER_CONVERSION_H
