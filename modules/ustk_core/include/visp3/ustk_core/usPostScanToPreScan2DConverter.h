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

/**
 * @file usPostScanToPreScan2DConverter.h
 * @brief 2D scan-converter
 */

#ifndef __usPostScanToPreScan2DConverter_h_
#define __usPostScanToPreScan2DConverter_h_

#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>

/**
 * @class usPostScanToPreScan2DConverter
 * @brief 2D back-scan converter
 * @ingroup module_ustk_core
 *
 * This class allows to convert 2D post-scan ultrasound images to pre-scan.
 * The convertion is applied in the convert() method.
 *
 * Here is an example of how to use the converter, to build a pre-scan image from a post-scan image.
 *
 * \code
#include <visp3/ustk_core/usPostScanToPreScan2DConverter.h>

int main()
{
  // example of 2D post-scan image settings
  unsigned int width = 320;
  unsigned int height = 240;
  double transducerRadius = 0.045;
  double scanLinePitch = 0.0012;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double widthResolution = 0.002;
  double heightResolution = 0.002;

  vpImage<unsigned char> I(height, width); // to fill
  usImagePostScan2D<unsigned char> postScan2d;
  postScan2d.setTransducerRadius(transducerRadius);
  postScan2d.setScanLinePitch(scanLinePitch);
  postScan2d.setScanLineNumber(scanLineNumber);
  postScan2d.setTransducerConvexity(isTransducerConvex);
  postScan2d.setWidthResolution(widthResolution);
  postScan2d.setHeightResolution(heightResolution);
  postScan2d.setData(I);

  usImagePreScan2D <unsigned char> preScan; // converter output
  usPostScanToPreScan2DConverter backConverter;
  backConverter.convert(postScan2d,preScan); // preScan is now an image built from postScan image
}
 * \endcode
 */
class VISP_EXPORT usPostScanToPreScan2DConverter
{
public:
  usPostScanToPreScan2DConverter();

  // initialisations constructors
  usPostScanToPreScan2DConverter(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
                                 const int scanLineNumber);
  usPostScanToPreScan2DConverter(const usTransducerSettings &transducerSettings, const int BModeSampleNumber,
                                 const int scanLineNumber, const double xResolution, const double yResolution);

  ~usPostScanToPreScan2DConverter();

  void convert(const usImagePostScan2D<unsigned char> &imageToConvert, usImagePreScan2D<unsigned char> &imageConverted,
               int preScanSamples);

protected:
  void init(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
            const int scanLineNumber);
  void init(const usTransducerSettings &inputSettings, const int BModeSampleNumber, const int scanLineNumber,
            const double xResolution, const double yResolution);

private:
  vpMatrix m_iMap;
  vpMatrix m_jMap;
  double m_xResolution;
  double m_yResolution;
  int m_scanLineNumber;
  int m_BModeSampleNumber;
  usTransducerSettings m_initSettings;

  bool m_isInit;

  double interpolateLinear(const vpImage<unsigned char> &I, double x, double y);
};

#endif // US_BACK_SCAN_CONVERTER_2D_H
