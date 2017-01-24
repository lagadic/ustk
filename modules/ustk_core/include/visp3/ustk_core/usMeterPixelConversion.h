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
* @file usMeterPixelConversion.h
* @brief Conversion between a position in meters in the space and the equivalent pixel position in the ultrasound image.
*/

#ifndef __usMeterPixelConversion_h_
#define __usMeterPixelConversion_h_

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
