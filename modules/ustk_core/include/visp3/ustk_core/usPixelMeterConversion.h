/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
* @file usPixelMeterConversion.h
* @brief Conversion between a pixel position in the ultrasound image and the real position in meters.
*/

#ifndef __usPixelMeterConversion_h_
#define __usPixelMeterConversion_h_

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>

/**
* @class usPixelMeterConversion
* @brief Conversion between a pixel position in the ultrasound image and the real position in meters.
* @ingroup module_ustk_core
*
* @warning Be sure you correctly filled your acquisition settings (probe radius, scan line pitch, etc...). Those parameters are used in the conversion !
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
