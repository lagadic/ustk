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

#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>
#include <visp/vpMath.h>

usPreScanToPostScan2DConverter::usPreScanToPostScan2DConverter() {}

usPreScanToPostScan2DConverter::~usPreScanToPostScan2DConverter() {}

/**
* Initialize the scan-converter.
* @param inputSettings Post-scan settings : transducer radius, pitch, depth, and resolutions.
* @param BModeSampleNumber Number of samples along a scan line : height of the pre-scan image to convert.
* @param scanLineNumber Number of scan lines : width of the pre-scan image to convert.
*/

void usPreScanToPostScan2DConverter::init(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
            const int scanLineNumber)
{

  //check resolution to avoir errors
  if(inputSettings.getHeightResolution() == 0.0 || inputSettings.getWidthResolution() == 0.0)
    throw(vpException(vpException::notInitialized, "Please fill the post-scan resplution before init the conversion."));

  m_scanLineNumber = scanLineNumber;
  m_BModeSampleNumber = BModeSampleNumber;
  m_xResolution = inputSettings.getWidthResolution();
  m_yResolution = inputSettings.getHeightResolution();
  m_settings = inputSettings;

  double APitch = inputSettings.getDepth() / (double)(BModeSampleNumber);
  double LPitch = inputSettings.getFieldOfView() * inputSettings.getTransducerRadius() / (double)(scanLineNumber -1);

  double r_min = inputSettings.getTransducerRadius();
  double r_max = (inputSettings.getTransducerRadius() + APitch * BModeSampleNumber);
  double t_min = - ( (double)(scanLineNumber-1) * LPitch) / (2.0 * inputSettings.getTransducerRadius());
  double t_max = - t_min;
  double x_min = r_min * cos(t_min);
  double x_max = r_max;
  double y_min = r_max * sin(t_min);
  double y_max = r_max * sin(t_max);

  m_height = vpMath::round((x_max - x_min) / m_yResolution);
  m_width = vpMath::round((y_max - y_min) / m_xResolution);

  m_rMap.resize(m_height, m_width);
  m_tMap.resize(m_height, m_width);

  double x, y;
  for (unsigned int i = 0; i < m_height; ++i) {
    for (unsigned int j = 0; j < m_width; ++j) {
      x = x_min + i * m_yResolution;
      y = y_min + j * m_xResolution;
      m_rMap[i][j] = (sqrt(x * x + y * y) - inputSettings.getTransducerRadius()) / APitch;
      m_tMap[i][j] = atan2(y, x) * inputSettings.getTransducerRadius() / LPitch + (scanLineNumber-1) / 2.0;
    }
  }
}

/**
* Initialize the scan-converter.
* @param inputSettings Post-scan settings : transducer radius, pitch, depth, and resolutions.
* @param BModeSampleNumber Number of samples along a scan line : height of the pre-scan image to convert.
* @param scanLineNumber Number of scan lines : width of the pre-scan image to convert.
* @param xResolution Size of a pixel in x direction of the post scan image built.
* @param yResolution Size of a pixel in y direction of the post scan image built.
*/

void usPreScanToPostScan2DConverter::init(const usTransducerSettings &inputSettings, const int BModeSampleNumber,
                             const int scanLineNumber, const double xResolution, const double yResolution)
{
  m_scanLineNumber = scanLineNumber;
  m_BModeSampleNumber = BModeSampleNumber;
  m_xResolution = xResolution;
  m_yResolution = yResolution;
  m_settings = inputSettings;

  double APitch = inputSettings.getDepth() / (double)(BModeSampleNumber);
  double LPitch = inputSettings.getFieldOfView() * inputSettings.getTransducerRadius() / (double)(scanLineNumber -1);

  double r_min = inputSettings.getTransducerRadius();
  double r_max = (inputSettings.getTransducerRadius() + APitch * BModeSampleNumber);
  double t_min = - ( (double)(scanLineNumber-1) * LPitch) / (2.0 * inputSettings.getTransducerRadius());
  double t_max = - t_min;
  double x_min = r_min * cos(t_min);
  double x_max = r_max;
  double y_min = r_max * sin(t_min);
  double y_max = r_max * sin(t_max);

  m_height = vpMath::round((x_max - x_min) / m_yResolution);
  m_width = vpMath::round((y_max - y_min) / m_xResolution);

  m_rMap.resize(m_height, m_width);
  m_tMap.resize(m_height, m_width);
  
  double x, y;
  for (unsigned int i = 0; i < m_height; ++i) {
    for (unsigned int j = 0; j < m_width; ++j) {
      x = x_min + i * m_yResolution;
      y = y_min + j * m_xResolution;
      m_rMap[i][j] = (sqrt(x * x + y * y) - inputSettings.getTransducerRadius()) / APitch;
      m_tMap[i][j] = atan2(y, x) * inputSettings.getTransducerRadius() / LPitch + (scanLineNumber-1) / 2.0;
    }
  }
}

/**
* Run the scan-converter.
* @param [out] postScanImage Post-scan image : result of the scan conversion.
* @param [in] preScanImage Pre-scan image to convert.
*/
void usPreScanToPostScan2DConverter::run(const usImagePreScan2D<unsigned char> &preScanImage, usImagePostScan2D<unsigned char> &postScanImage)
{
  postScanImage.resize(m_height, m_width);
  for (unsigned int i = 0; i < m_height; ++i)
    for (unsigned int j = 0; j < m_width; ++j) {
      double u = m_rMap[i][j];
      double v = m_tMap[i][j];
      postScanImage(i, j, (unsigned char)interpolateLinear(preScanImage, u, v));
    }

  //saving settings in postScanImage
  postScanImage.setHeightResolution(m_yResolution);
  postScanImage.setWidthResolution(m_xResolution);
  postScanImage.setScanLineNumber(m_scanLineNumber);
  postScanImage.setTransducerConvexity(preScanImage.isTransducerConvex());
  postScanImage.setScanLinePitch(m_settings.getScanLinePitch());
  postScanImage.setTransducerRadius(m_settings.getTransducerRadius());
}


double usPreScanToPostScan2DConverter::interpolateLinear(const vpImage<unsigned char>& I, double x, double y)
{
  int x1 = (int)floor(x);
  int x2 = (int)ceil(x);
  int y1 = (int)floor(y);
  int y2 = (int)ceil(y);

  if ((0 <= x) && (x < I.getHeight()) && (0 <= y) && (y < I.getWidth())) {
    double val1, val2;
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
