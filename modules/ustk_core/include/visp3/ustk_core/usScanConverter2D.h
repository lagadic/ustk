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

/**
 * @file usScanConverter2D.h
 * @brief 2D scan-converter
 * @author Pierre Chatelain
 */

#ifndef US_SCAN_CONVERTER_2D_H
#define US_SCAN_CONVERTER_2D_H

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
