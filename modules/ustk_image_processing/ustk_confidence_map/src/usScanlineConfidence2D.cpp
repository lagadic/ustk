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
 * Pierre Chatelain
 * Alexandre Krupa
 *
 *****************************************************************************/

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>

/**
* Default constructor.
*/
usScanlineConfidence2D::usScanlineConfidence2D() {}

/**
* Default destructor.
*/
usScanlineConfidence2D::~usScanlineConfidence2D() {}

/**
* Run the confidence map processor on pre-scan image.
* @param [out] preScanConfidence Confidence image computed.
* @param [in] preScanImage Pre-scan image to process.
*/
void usScanlineConfidence2D::run(usImagePreScan2D<unsigned char> &preScanConfidence, const usImagePreScan2D<unsigned char> &preScanImage)
{
  preScanConfidence.setImagePreScanSettings(preScanImage);
  unsigned int AN = preScanImage.getHeight();
  unsigned int LN = preScanImage.getWidth();
  if(AN == 0 || LN == 0)
    throw(vpException(vpException::notInitialized, "pre-scan image dimension is 0"));

  preScanConfidence.resize(AN, LN);

  // Find min and max
  double min = 255.0;
  double max = 0.0;
  for (unsigned int i = 0; i < AN; ++i)
    for (unsigned int j = 0; j < LN; ++j) {
      if (static_cast<double>(preScanImage(i,j)) < min)
        min = static_cast<double>(preScanImage(i,j));
      if (static_cast<double>(preScanImage(i,j)) > max)
        max = static_cast<double>(preScanImage(i,j));
    }

  // Integrate scan lines
  for (unsigned int j = 0; j < LN; ++j) {
    double sum = 0.0;
    for (unsigned int i = 0; i < AN; ++i)
      sum += vpMath::sqr((preScanImage(i,j) - min) / (max - min));

    double val = sum;
    preScanConfidence(0, j, 255);
    for (unsigned int i = 1; i < AN; ++i) {
      val -= vpMath::sqr((preScanImage(i-1, j) - min) / (max - min));
      preScanConfidence(i, j, (unsigned char)(val / sum * 255));
    }
  }
}
