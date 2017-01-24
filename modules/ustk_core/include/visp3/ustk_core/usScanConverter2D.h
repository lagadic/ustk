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
 * @file usScanConverter2D.h
 * @brief 2D scan-converter
 */

#ifndef __usScanConverter2D_h_
#define __usScanConverter2D_h_

#include <visp3/ustk_core/usScanConverter2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>

/**
 * @class usScanConverter2D
 * @brief 2D scan-converter
 * @ingroup module_ustk_core
 *
 * This class allows to convert 2D pre-scan ultrasound images to post-scan.
 * The converter should be initialized through init() and then applied through run().
 */
class VISP_EXPORT usScanConverter2D
{
 public:

  usScanConverter2D();

  ~usScanConverter2D();

  void init(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
            const int scanLineNumber);

  void init(const usTransducerSettings &inputSettings, const int BModeSampleNumber,
            const int scanLineNumber, const double xResolution, const double yResolution);

  void run(const usImagePreScan2D<unsigned char> &preScanImage, usImagePostScan2D<unsigned char> &postScanImage);

 private:
  double interpolateLinear(const vpImage<unsigned char>& I, double x, double y);

  vpMatrix m_rMap;
  vpMatrix m_tMap;

  double m_xResolution;
  double m_yResolution;
  int m_scanLineNumber;
  int m_BModeSampleNumber;

  usTransducerSettings m_settings;

  unsigned int m_height;
  unsigned int m_width;

};

#endif // US_SCAN_CONVERTER_2D_H
