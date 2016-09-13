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
 *
 *****************************************************************************/

/**
 * @file usImagePreScan2D.h
 * @brief 2D prescan ultrasound image.
 * @author Pierre Chatelain
 */

#ifndef US_IMAGE_PRESCAN_2D_H
#define US_IMAGE_PRESCAN_2D_H

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImageSettings.h>

/**
 * @class usImagePreScan2D
 * @brief 2D prescan ultrasound image.
 * @author Pierre Chatelain
 *
 * This class represents a 2D ultrasound prescan frame.
 */
class VISP_EXPORT usImagePreScan2D : public usImageSettings, public vpImage<unsigned char> {
 public:
  /**
   * Constructor.
   */
  usImagePreScan2D();

  /**
   * Initializing constructor.
   */
  usImagePreScan2D(unsigned int AN, unsigned int LN);

  /**
   * Initializing constructor.
   */
  usImagePreScan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
		  float resolution, float BSampleFreq, float probeElementPitch);

  /**
   * Copy constructor.
   */
  usImagePreScan2D(const usImagePreScan2D &other, const bool copy=true);

  /**
   * Destructor.
   */
  ~usImagePreScan2D();

  void copyFrom(const vpImage<unsigned char> &I);

  /**
   * Get the number of A-samples in a line.
   */
  unsigned int getAN() const;

  /**
   * Get the number of lines.
   */
  unsigned int getLN() const;

};

#endif // US_IMAGE_PRESCAN_2D_H
