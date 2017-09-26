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
 *
 *****************************************************************************/

#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>
#include <visp/vpMath.h>

usPreScanToPostScan2DConverter::usPreScanToPostScan2DConverter() : m_initDone(false) {}

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
    throw(vpException(vpException::notInitialized, "Please fill the post-scan resolution before init the conversion."));

  if(inputSettings.isTransducerConvex()) {
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
  else{
    m_scanLineNumber = scanLineNumber;
    m_BModeSampleNumber = BModeSampleNumber;
    m_xResolution = inputSettings.getWidthResolution();
    m_yResolution = inputSettings.getHeightResolution();
    m_settings = inputSettings;

    double APitch = inputSettings.getDepth() / (double)(BModeSampleNumber);
    m_height = vpMath::round((APitch * BModeSampleNumber) / m_yResolution);
    m_width = vpMath::round( inputSettings.getScanLinePitch() * (scanLineNumber - 1) / m_xResolution);

    m_rMap.resize(m_height, m_width);
    m_tMap.resize(m_height, m_width);

    for (unsigned int i = 0; i < m_height; ++i) {
      for (unsigned int j = 0; j < m_width; ++j) {
        m_rMap[i][j] = j;
        m_tMap[i][j] = i;
      }
    }
  }

  m_initDone = true;
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
  if(inputSettings.isTransducerConvex()) {
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
  else{
    m_scanLineNumber = scanLineNumber;
    m_BModeSampleNumber = BModeSampleNumber;
    m_xResolution = xResolution;
    m_yResolution = yResolution;
    m_settings = inputSettings;

    double APitch = inputSettings.getDepth() / (double)(BModeSampleNumber);
    m_height = vpMath::round(inputSettings.getDepth() / m_yResolution);
    m_width = vpMath::round( inputSettings.getScanLinePitch() * (scanLineNumber - 1) / m_xResolution);

    m_rMap.resize(m_height, m_width);
    m_tMap.resize(m_height, m_width);

    double ratio1 = m_xResolution / inputSettings.getScanLinePitch();
    double ratio2 = m_yResolution/APitch ;

    for (unsigned int i = 0; i < m_height; ++i) {
      for (unsigned int j = 0; j < m_width; ++j) {
        m_rMap[i][j] = i * ratio2;
        m_tMap[i][j] = j * ratio1;
      }
    }
  }

  m_initDone = true;
}

/**
* Run the scan-converter.
* @param [in, out] postScanImage Post-scan image : result of the scan conversion.
* @param [in] preScanImage Pre-scan image to convert.
* @param [in] xResolution Size of a pixel along x axis in post-scan image built (optionnal).
* @param [in] yResolution Size of a pixel along y axis in post-scan image built (optionnal).
*/
void usPreScanToPostScan2DConverter::convert(const usImagePreScan2D<unsigned char> &preScanImage, usImagePostScan2D<unsigned char> &postScanImage,double xResolution, double yResolution)
{
  // if user specified the resolution wanted
  if(xResolution != 0. && yResolution != 0.) {
    init(preScanImage, preScanImage.getBModeSampleNumber(), preScanImage.getScanLineNumber(), xResolution, yResolution);
  }

  //check if init is done
  if (!m_initDone) {
    if(preScanImage.isTransducerConvex()) {
      double resolution = preScanImage.getAxialResolution();
      init(preScanImage, preScanImage.getBModeSampleNumber(), preScanImage.getScanLineNumber(), resolution, resolution);
    }
    else {
      init(preScanImage, preScanImage.getBModeSampleNumber(), preScanImage.getScanLineNumber(), preScanImage.getScanLinePitch(), preScanImage.getAxialResolution());
    }
  }

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
