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

#include <visp3/ustk_confidence_map/usScanlineConf2D.h>

usScanlineConf2D::usScanlineConf2D() {}

void usScanlineConf2D::init(usScanLineConfidenceType type)
{
  m_type = type;
}

void usScanlineConf2D::run(vpImage<unsigned char> &Dst, const vpImage<unsigned char> &Src)
{
  unsigned int AN = Src.getHeight();
  unsigned int LN = Src.getWidth();
  double coef = 0.9;
  double attenuation = 0.0;

  Dst.resize(AN, LN);

  switch(m_type) {
  case US_CONF_MAX: {
    // Find min intensity
    unsigned char min = 255;
    for (unsigned int i = 0; i < AN; ++i)
      for (unsigned int j = 0; j < LN; ++j)
        if (Src(i,j) * exp(- attenuation * i) < min) min = Src(i,j) * exp(- attenuation * i);
    
    for (unsigned int j = 0; j < LN; ++j)
      Dst(AN-1, j, Src(AN-1, j) * exp(- attenuation * (LN - 1)) - min);
    
    for (int i = AN - 2; i >= 0; --i) {
      unsigned char val = Src(i, 0) * exp(- attenuation * i) - min;
      Dst(i, 0, vpMath::maximum(val, static_cast<unsigned char>(1.0 * Dst(i+1, 0))));
      Dst(i, 0, vpMath::maximum(Dst(i, 0), static_cast<unsigned char>(coef * Dst(i+1, 1))));
      for (unsigned int j = 1; j < LN - 1; ++j) {
        val = Src(i, j) * exp(- attenuation * i) - min;
        Dst(i, j, vpMath::maximum(val, static_cast<unsigned char>(1.0 * Dst(i+1, j))));
        Dst(i, j, vpMath::maximum(Dst(i, j), static_cast<unsigned char>(coef * Dst(i+1, j-1))));
        Dst(i, j, vpMath::maximum(Dst(i, j), static_cast<unsigned char>(coef * Dst(i+1, j+1))));
      }
      val = Src(i, LN-1) * exp(- attenuation * i) - min;
      Dst(i, LN-1, vpMath::maximum(val, Dst(i+1, LN-1)));
      Dst(i, LN-1, vpMath::maximum(Dst(i, LN-1), static_cast<unsigned char>(coef * Dst(i+1, LN-2))));
    }

    float max = 0.0;
    
    for (unsigned int j = 0; j < LN; ++j) {
      float val = static_cast<float>(Dst(0, j));
      if (val > max) max = val;
      float scale = 255.0f / val;
      for (unsigned int i = 0; i < AN; ++i) {
        Dst(i, j, static_cast<float>(Dst(i, j)) * scale);
      }
    }
  } break;
  case US_CONF_INTEGRATION: {
    // Find min and max
    double min = 255.0;
    double max = 0.0;
    for (unsigned int i = 0; i < AN; ++i)
      for (unsigned int j = 0; j < LN; ++j) {
        if (static_cast<double>(Src(i,j)) < min)
          min = static_cast<double>(Src(i,j));
        if (static_cast<double>(Src(i,j)) > max)
          max = static_cast<double>(Src(i,j));
      }

    
    // Integrate scan lines
    for (unsigned int j = 0; j < LN; ++j) {
      double sum = 0.0;
      for (unsigned int i = 0; i < AN; ++i)
        sum += vpMath::sqr((Src(i,j) - min) / (max - min));

      double val = sum;
      Dst(0, j, 255);
      for (unsigned int i = 1; i < AN; ++i) {
        val -= vpMath::sqr((Src(i-1, j) - min) / (max - min));
        Dst(i, j, val / sum * 255);
      }
    }
  }
  }
}
