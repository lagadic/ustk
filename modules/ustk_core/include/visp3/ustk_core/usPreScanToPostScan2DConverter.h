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
 * @file usPreScanToPostScan2DConverter.h
 * @brief 2D scan-converter
 */

#ifndef __usPreScanToPostScan2DConverter_h_
#define __usPreScanToPostScan2DConverter_h_

#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>

/**
 * @class usPreScanToPostScan2DConverter
 * @brief 2D scan-converter
 * @ingroup module_ustk_core
 *
 * This class allows to convert 2D pre-scan ultrasound images to post-scan images.
 * The convertion is applied in the convert() method.
 *
 * Considering the following usImagePreScan2D image (convex or linear) as input:
 * \image html img-usImagePreScan2D.png
 * this class generates an usImagePostScan2D (convex or linear)  as output:
 * \image html img-usImagePostScan2D.png
 *
 * Here is an example of how to use the converter, to build a post-scan image from a pre-scan image.
 *
 * \code
 * #include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>

int main()
{
  // example of 2D pre-scan image settings
  unsigned int width = 320;
  unsigned int height = 240;
  double transducerRadius = 0.045;
  double scanLinePitch = 0.0012;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double axialResolution = 0.002;

  vpImage<unsigned char> I(height, width);
  usImagePreScan2D <unsigned char> preScan; // your input pre-scan image
  // then you can fill the preScan image and settings
  preScan.setTransducerRadius(transducerRadius);
  preScan.setScanLinePitch(scanLinePitch);
  preScan.setScanLineNumber(scanLineNumber);
  preScan.setTransducerConvexity(isTransducerConvex);
  preScan.setAxialResolution(axialResolution);
  preScan.setData(I);

  // converter output
  usImagePostScan2D<unsigned char> postScan;
  usPreScanToPostScan2DConverter scanConverter;
  scanConverter.convert(preScan,postScan,0.0005,0.0005); // now postScan is filled from preScan, with pixels of 0.5mm
}
 * \endcode
 *
 */
class VISP_EXPORT usPreScanToPostScan2DConverter
{
  friend class usRFToPostScan2DConverter;

public:
  usPreScanToPostScan2DConverter();

  ~usPreScanToPostScan2DConverter();

  void convert(const usImagePreScan2D<unsigned char> &preScanImage, usImagePostScan2D<unsigned char> &postScanImage,
               double xResolution = 0., double yResolution = 0.);

protected:
  void init(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
            const int scanLineNumber);

  void init(const usTransducerSettings &inputSettings, const int BModeSampleNumber, const int scanLineNumber,
            const double xResolution, const double yResolution);

private:
  double interpolateLinear(const vpImage<unsigned char> &I, double x, double y);

  vpMatrix m_rMap;
  vpMatrix m_tMap;

  double m_xResolution;
  double m_yResolution;
  int m_scanLineNumber;
  int m_BModeSampleNumber;

  usTransducerSettings m_settings;

  unsigned int m_height;
  unsigned int m_width;

  bool m_initDone;
};

#endif // US_SCAN_CONVERTER_2D_H
