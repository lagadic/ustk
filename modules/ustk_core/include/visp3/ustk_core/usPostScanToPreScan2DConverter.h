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

#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>


/**
 * @class usPostScanToPreScan2DConverter
 * @brief 2D back-scan converter
 * @ingroup module_ustk_core
 *
 * This class allows to convert 2D post-scan ultrasound images to pre-scan.
 * The converter should be initialized through init() and then applied through run().
 */
class VISP_EXPORT usPostScanToPreScan2DConverter
{
 public:

  usPostScanToPreScan2DConverter();

  //initialisations constructors
  usPostScanToPreScan2DConverter(const usImagePostScan2D<unsigned char> &inputSettings,
  const int BModeSampleNumber, const int scanLineNumber);
  usPostScanToPreScan2DConverter(const usTransducerSettings &transducerSettings,
  const int BModeSampleNumber, const int scanLineNumber,const double xResolution, const double yResolution);

  ~usPostScanToPreScan2DConverter();

  void init(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber, const int scanLineNumber);
  void init(const usTransducerSettings &inputSettings, const int BModeSampleNumber,
            const int scanLineNumber,const double xResolution, const double yResolution);

  void run(const usImagePostScan2D<unsigned char> &imageToConvert, usImagePreScan2D<unsigned char> &imageConverted);

 private:
  vpMatrix m_iMap;
  vpMatrix m_jMap;
  double m_xResolution;
  double m_yResolution;
  int m_scanLineNumber;
  int m_BModeSampleNumber;
  usTransducerSettings m_initSettings;

  double interpolateLinear(const vpImage<unsigned char>& I, double x, double y);
};

#endif // US_BACK_SCAN_CONVERTER_2D_H
