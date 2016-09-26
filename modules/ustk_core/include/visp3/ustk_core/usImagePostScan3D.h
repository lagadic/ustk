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
* @file usImagePostScan3D.h
* @brief 3D postscan ultrasound image.
*/

#ifndef US_IMAGE_POSTSCAN_3D_H
#define US_IMAGE_POSTSCAN_3D_H

#include <visp3/core/vpConfig.h>
#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usImageSettings3D.h>

/**
* @class usImagePostScan3D
* @brief 3D postscan ultrasound image.
*
* This class represents a 3D ultrasound postscan frame.
*/
class VISP_EXPORT usImagePostScan3D : public usImage3D<unsigned char>, public usImageSettings3D {
public:
  usImagePostScan3D();

  usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isImageConvex, bool isMotorConvex);

  usImagePostScan3D(const usImagePostScan3D &other);

  usImagePostScan3D(const usImage3D<unsigned char> &other);

  usImagePostScan3D(const usImageSettings3D &other);

  usImagePostScan3D(const usImage3D<unsigned char> &otherImage, const usImageSettings3D &otherSettings);

  ~usImagePostScan3D();

  void setWidthResolution(double widthResolution);

  double getWidthResolution();

  void setHeightResolution(double widthResolution);

  double getHeightResolution();

private:
  double m_widthResolution;
  double m_heightResolution;
};

#endif // US_IMAGE_POSTSCAN_3D_H
