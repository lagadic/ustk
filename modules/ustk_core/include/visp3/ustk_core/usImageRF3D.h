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
* @file usImageRF3D.h
* @brief 3D RF ultrasound image.
*/

#ifndef US_IMAGE_RF_2D_H
#define US_IMAGE_RF_2D_H

#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usImageSettings3D.h>

/**
* @class usImageRF3D
* @brief 3D Rf ultrasound image.
*
* This class represents a 3D ultrasound RF frame.
*/
class VISP_EXPORT usImageRF3D : public usImage3D<short>, public usImageSettings3D {
public:

  usImageRF3D();

  usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN);

  usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN,
    float probeRadius, float motorRadius, float lineAngle, float frameAngle,
    float resolution, float BSampleFreq, float probeElementPitch);

  usImageRF3D(const usImageRF3D &other);

  ~usImageRF3D();

  unsigned int getAN() const;

  unsigned int getLN() const;

  unsigned int getFN() const;
};

#endif // US_IMAGE_RF_2D_H
