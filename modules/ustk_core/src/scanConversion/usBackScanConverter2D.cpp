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

//#include <visp/vpMath.h>

/**
 * Default constructor.
 */
usBackScanConverter2D::usBackScanConverter2D() {}

/**
 * Initialisation constructor.
* @param inputSettings Post-scan settings you want to use to back-convert image.
* @param BModeSampleNumber Number of samples along a scan line : height of the pre-scan image built by run().
* @param scanLineNumber Number of scan lines : width of the pre-scan image built by run().
 */
usBackScanConverter2D::usBackScanConverter2D(const usImagePostScan2D<unsigned char> &inputSettings,
 const int BModeSampleNumber, const int scanLineNumber)
 {
 init(inputSettings,BModeSampleNumber,scanLineNumber);
 }

/**
 * Default constructor.
 */
usBackScanConverter2D::usBackScanConverter2D(const usTransducerSettings &transducerSettings,
 const int BModeSampleNumber, const int scanLineNumber,const double xResolution, const double yResolution)
{
init(transducerSettings, BModeSampleNumber, scanLineNumber, xResolution, yResolution);
}

/**
 * Destructor.
 */
usBackScanConverter2D::~usBackScanConverter2D() {}

/**
* Initialize the back-scan converter.
* @param inputSettings Post-scan settings you want to use to back-convert image.
* @param BModeSampleNumber Number of samples along a scan line : height of the pre-scan image built by run().
* @param scanLineNumber Number of scan lines : width of the pre-scan image built by run().
*/
void usBackScanConverter2D::init(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
                                 const int scanLineNumber)
{
  //check resolution to avoir errors
  if(inputSettings.getHeightResolution() == 0.0 || inputSettings.getWidthResolution() == 0.0)
    throw(vpException(vpException::notInitialized, "Please fill the post-scan resplution before init the conversion."));


  //convex transducer scan conversion
  if(inputSettings.isTransducerConvex()) {
    double APitch = inputSettings.getDepth() / (double)(BModeSampleNumber);
    double LPitch = inputSettings.getScanLinePitch() * inputSettings.getTransducerRadius();

    double r_min = inputSettings.getTransducerRadius();
    double r_max = (inputSettings.getTransducerRadius() + APitch * BModeSampleNumber);
    double t_min = - inputSettings.getFieldOfView() / 2.0;
    //double t_max = - t_min;
    double x_min = r_min * cos(t_min);
    //double x_max = r_max;
    double y_min = r_max * sin(t_min);
    //double y_max = r_max * sin(t_max);

    m_iMap.resize(BModeSampleNumber, scanLineNumber);
    m_jMap.resize(BModeSampleNumber, scanLineNumber);

    double r, t;
    for (int u = 0; u < BModeSampleNumber; ++u) {
      for (int v = 0; v < scanLineNumber; ++v) {
        r = inputSettings.getTransducerRadius() + APitch * u;
        t = (v - (scanLineNumber - 1) / 2.0) * LPitch / inputSettings.getTransducerRadius();
        m_iMap[u][v] = (r * cos(t) - x_min) / inputSettings.getHeightResolution(); //resolution to check
        m_jMap[u][v] = (r * sin(t) - y_min) / inputSettings.getWidthResolution(); //resolution to check
      }
    }
  }
  //linear transducer scan-conversion
  else {
    //not implemented
  }
  //saving settings
  m_initSettings = inputSettings;
  m_xResolution = inputSettings.getWidthResolution();
  m_yResolution = inputSettings.getHeightResolution();
  m_scanLineNumber = scanLineNumber;
  m_BModeSampleNumber = BModeSampleNumber;
}

/**
* Initialize the back-scan converter.
* @param transducerSettings Transducer settings of the pre-scan image.
* @param xResolution Height of a pixel
* @param yResolution Width of a pixel.
* @param BModeSampleNumber Number of pre-scan samples you want in output of back conversion.
* @param scanLineNumber Number of scan lines you want in output of back conversion.
*/
void usBackScanConverter2D::init(const usTransducerSettings &transducerSettings, const int BModeSampleNumber,
 const int scanLineNumber,const double xResolution, const double yResolution)
{
  //convex transducer scan conversion
  if(transducerSettings.isTransducerConvex()) {
    double APitch = transducerSettings.getDepth() / BModeSampleNumber;
    double LPitch = transducerSettings.getFieldOfView() * transducerSettings.getTransducerRadius() / (scanLineNumber -1);

    double r_min = transducerSettings.getTransducerRadius();
    double r_max = (transducerSettings.getTransducerRadius() + APitch * BModeSampleNumber);
    double t_min = - transducerSettings.getFieldOfView() / 2.0;
    //double t_max = - t_min;
    double x_min = r_min * cos(t_min);
    //double x_max = r_max;
    double y_min = r_max * sin(t_min);
    //double y_max = r_max * sin(t_max);

    m_iMap.resize(BModeSampleNumber, scanLineNumber);
    m_jMap.resize(BModeSampleNumber, scanLineNumber);

    double r, t;
    for (int u = 0; u < BModeSampleNumber; ++u) {
      for (int v = 0; v < scanLineNumber; ++v) {
        r = transducerSettings.getTransducerRadius() + APitch * u;
        t = (v - (scanLineNumber - 1) / 2.0) * LPitch / transducerSettings.getTransducerRadius();
        m_iMap[u][v] = (r * cos(t) - x_min) / yResolution; //to check
        m_jMap[u][v] = (r * sin(t) - y_min) / xResolution; //to check
      }
    }
  }
  //linear transducer scan-conversion
  else {
    //not implemented
  }
  //saving settings
  m_initSettings = transducerSettings;
  m_xResolution = xResolution;
  m_yResolution = yResolution;
  m_scanLineNumber = scanLineNumber;
  m_BModeSampleNumber = BModeSampleNumber;
}

/**
* Run the back-scan converter.
* @param [in] imageToConvert Post-scan image to convert back.
* @param [out] imageConverted Pre-scan image obtained after back conversion.
*/
void usBackScanConverter2D::run(const usImagePostScan2D<unsigned char> &imageToConvert, usImagePreScan2D<unsigned char> &imageConverted)
{
  imageConverted.setImagePreScanSettings(usImagePreScanSettings(m_initSettings, m_yResolution));


  imageConverted.setScanLineNumber(m_scanLineNumber);
  imageConverted.setFieldOfView(m_initSettings.getFieldOfView());
  imageConverted.setAxialResolution(m_initSettings.getDepth()/m_BModeSampleNumber);
  imageConverted.setTransducerConvexity(m_initSettings.isTransducerConvex());
  imageConverted.setTransducerRadius(m_initSettings.getTransducerRadius());

  imageConverted.resize(m_BModeSampleNumber,m_scanLineNumber);
  double i, j;
  for (unsigned int u = 0; u < imageConverted.getBModeSampleNumber(); ++u)
    for (unsigned int v = 0; v < imageConverted.getScanLineNumber(); ++v) {
      i = m_iMap[u][v];
      j = m_jMap[u][v];
      imageConverted[u][v] = static_cast<unsigned char>(interpolateLinear(imageToConvert, i, j));
    }
}

double usBackScanConverter2D::interpolateLinear(const vpImage<unsigned char>& I, double x, double y)
{
  int x1 = (int)floor(x);
  int x2 = (int)ceil(x);
  int y1 = (int)floor(y);
  int y2 = (int)ceil(y);
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

