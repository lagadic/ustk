/****************************************************************************
 *
 * This file is part of the UsConfidenceMaps software.
 * Copyright (C) 2013 - 2016 by Inria. All rights reserved.
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
* Initialize the confidence map processor.
* @param type Type of scanline confidence to compute.
*/
void usScanlineConfidence2D::init(usScanLineConfidenceType type)
{
  m_type = type;
}

/**
* Run the confidence map processor on pre-scan image.
* @param [out] preScanConfidence Confidence image computed.
* @param [in] preScanImage Pre-scan image to process.
*/
void usScanlineConfidence2D::run(usImagePreScan2D<unsigned char> &preScanConfidence, const usImagePreScan2D<unsigned char> &preScanImage)
{
  unsigned int AN = preScanImage.getHeight();
  unsigned int LN = preScanImage.getWidth();
  double coef = 0.9;
  double attenuation = 0.0;

  preScanConfidence.resize(AN, LN);

  switch(m_type) {
  case US_CONF_MAX: {
    // Find min intensity
    unsigned char min = 255;
    for (unsigned int i = 0; i < AN; ++i)
      for (unsigned int j = 0; j < LN; ++j)
        if (preScanImage(i,j) * exp(- attenuation * i) < min) min = preScanImage(i,j) * exp(- attenuation * i);
    
    for (unsigned int j = 0; j < LN; ++j)
      preScanConfidence(AN-1, j, preScanImage(AN-1, j) * exp(- attenuation * (LN - 1)) - min);
    
    for (int i = AN - 2; i >= 0; --i) {
      unsigned char val = preScanImage(i, 0) * exp(- attenuation * i) - min;
      preScanConfidence(i, 0, vpMath::maximum(val, static_cast<unsigned char>(1.0 * preScanConfidence(i+1, 0))));
      preScanConfidence(i, 0, vpMath::maximum(preScanConfidence(i, 0), static_cast<unsigned char>(coef * preScanConfidence(i+1, 1))));
      for (unsigned int j = 1; j < LN - 1; ++j) {
        val = preScanImage(i, j) * exp(- attenuation * i) - min;
        preScanConfidence(i, j, vpMath::maximum(val, static_cast<unsigned char>(1.0 * preScanConfidence(i+1, j))));
        preScanConfidence(i, j, vpMath::maximum(preScanConfidence(i, j), static_cast<unsigned char>(coef * preScanConfidence(i+1, j-1))));
        preScanConfidence(i, j, vpMath::maximum(preScanConfidence(i, j), static_cast<unsigned char>(coef * preScanConfidence(i+1, j+1))));
      }
      val = preScanImage(i, LN-1) * exp(- attenuation * i) - min;
      preScanConfidence(i, LN-1, vpMath::maximum(val, preScanConfidence(i+1, LN-1)));
      preScanConfidence(i, LN-1, vpMath::maximum(preScanConfidence(i, LN-1), static_cast<unsigned char>(coef * preScanConfidence(i+1, LN-2))));
    }

    float max = 0.0;
    
    for (unsigned int j = 0; j < LN; ++j) {
      float val = static_cast<float>(preScanConfidence(0, j));
      if (val > max) max = val;
      float scale = 255.0f / val;
      for (unsigned int i = 0; i < AN; ++i) {
        preScanConfidence(i, j, static_cast<float>(preScanConfidence(i, j)) * scale);
      }
    }
  } break;
  case US_CONF_INTEGRATION: {
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
        preScanConfidence(i, j, val / sum * 255);
      }
    }
  }
  }
}
