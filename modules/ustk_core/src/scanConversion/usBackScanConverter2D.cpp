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

#include <visp3/ustk_core/usBackScanConverter2D.h>


#include <visp/vpMath.h>

usBackScanConverter2D::usBackScanConverter2D() {}

usBackScanConverter2D::~usBackScanConverter2D() {}

/**
* Initialize the back-scan converter.
* @param imageToConvert Post-scan image to convert back.
* @param depth Distance between the first and the last pixel of a scanline, in meters.
* @param AN Number of samples in a scanline in the pre-scan image.
* @param resolution Size of a pixel (in meters).
*/
void usBackScanConverter2D::init(const usImagePostScan2D<unsigned char> imageToConvert, double depth, unsigned int AN, double resolution)
{
  m_postScanImage = imageToConvert;

  //convex transducer scan conversion
  if(imageToConvert.isTransducerConvex()) {
    double fov = (m_postScanImage.getScanLineNumber() - 1) * m_postScanImage.getScanLinePitch();
    double APitch = depth / AN;
    double LPitch = fov * m_postScanImage.getTransducerRadius() / (m_postScanImage.getScanLineNumber()-1);

    double r_min = imageToConvert.getTransducerRadius();
    double r_max = (imageToConvert.getTransducerRadius() + APitch * AN);
    double t_min = - fov / 2.0;
    double t_max = - t_min;
    double x_min = r_min * cos(t_min);
    double x_max = r_max;
    double y_min = r_max * sin(t_min);
    double y_max = r_max * sin(t_max);

    std::cout << "r_min " << r_min << std::endl;
    std::cout << "r_max " << r_max << std::endl;
    std::cout << "t_min " <<  t_min<< std::endl;
    std::cout << "t_max " << t_max << std::endl;
    std::cout << "x_min" << x_min << std::endl;
    std::cout << "x_max " << x_max << std::endl;
    std::cout << "y_min " << y_min << std::endl;
    std::cout << "y_max " << y_max << std::endl;

    m_iMap.resize(AN, m_postScanImage.getScanLineNumber());
    m_jMap.resize(AN, m_postScanImage.getScanLineNumber());

    double r, t;
    for (unsigned int u = 0; u < AN; ++u) {
      for (unsigned int v = 0; v < m_postScanImage.getScanLineNumber(); ++v) {
        r = m_postScanImage.getTransducerRadius() + APitch * u;
        t = (v - (m_postScanImage.getScanLineNumber() - 1) / 2.0) * LPitch / m_postScanImage.getTransducerRadius();
        m_iMap[u][v] = (r * cos(t) - x_min) / resolution;
        m_jMap[u][v] = (r * sin(t) - y_min) / resolution;
      }
    }
    //saving pre-scan dimensions
    m_preScanImage.resize(AN,m_postScanImage.getScanLineNumber());
    //saving settings
    m_preScanImage.setAxialResolution(APitch);
    m_preScanImage.setScanLinePitch(m_postScanImage.getScanLinePitch());
    m_preScanImage.setTransducerConvexity(true);
    m_preScanImage.setTransducerRadius(m_postScanImage.getTransducerRadius());

    std::cout << "backScan conversion parameters :" << std::endl;
    std::cout << "AN : " << AN << std::endl;
    std::cout << "LN : " << m_postScanImage.getScanLineNumber() << std::endl;
    std::cout << "line pitch : " << m_postScanImage.getScanLinePitch() << std::endl;
    std::cout << "LN : " << m_postScanImage.getScanLineNumber() << std::endl;
    std::cout << "radius : " << m_postScanImage.getTransducerRadius() << std::endl;
    std::cout << "APitch : " << APitch << std::endl;
    std::cout << "LPitch : " << LPitch << std::endl;
    std::cout << "resolution : " << resolution << std::endl;
  }
  //linear transducer scan-conversion
  else {
    //not implemented
  }
}

/**
* Run the back-scan converter.
* @param [out] imageConverted Pre-scan image obtained after back conversion.
*/
void usBackScanConverter2D::run(usImagePreScan2D<unsigned char> &imageConverted)
{
  imageConverted = m_preScanImage;
  double i, j;
  for (unsigned int u = 0; u < imageConverted.getBModeSampleNumber(); ++u)
    for (unsigned int v = 0; v < imageConverted.getScanLineNumber(); ++v) {
      i = m_iMap[u][v];
      j = m_jMap[u][v];
      imageConverted[u][v] = interpolateLinear(m_postScanImage, i, j);
    }
}

double usBackScanConverter2D::interpolateLinear(const vpImage<unsigned char>& I, double x, double y)
{
  int x1 = floor(x);
  int x2 = ceil(x);
  int y1 = floor(y);
  int y2 = ceil(y);
  double val1, val2;

  if ((0 <= x) && (x < I.getHeight()) && (0 <= y) && (y < I.getWidth())) {
    // Check whether the indices are within the image extent
    if (x1 < 0) ++x1;
    if (y1 < 0) ++y1;
    if (x2 >= static_cast<int>(I.getHeight())) --x2;
    if (y2 >= static_cast<int>(I.getWidth())) --y2;

    // Check whether the target is on the grid
    if (x1==x2) {
      val1 = I(x1, y1);
      val2 = I(x1, y2);
    }
    else {
      val1 = (x2 - x) * I(x1, y1) + (x - x1) * I(x2, y1);
      val2 = (x2 - x) * I(x1, y2) + (x - x1) * I(x2, y2);
    }
    if (y1==y2)
      return val1;
    else
      return (y2 - y) * val1 + (y - y1) * val2;
  }
  return 0.0;
}

