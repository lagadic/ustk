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
 * @author Pierre Chatelain
 *
 * This class allows to convert 2D prescan ultrasound images to postscan.
 * The converter should be initialized through init() and then applied through run().
 */
class VISP_EXPORT usScanConverter2D
{
 public:
  /// Constructor
  usScanConverter2D();

  /// Destructor
  ~usScanConverter2D();

  /// Check whether the probe is curved.
  bool isCurved() const;

  /// Get the probe radius (in meters).
  double getRadius() const;

  /// Get the imaging depth (in meters).
  double getDepth() const;

  /// Get the spacing between two samples along an A-line (in meters).
  double getAPitch() const;

  /// Get the spacing between two transducer elements (in meters).
  double getLPitch() const;

  /// Get the postscan resolution (in meters per pixel).
  double getResolution() const;

  /// Get the minimum x-coordinate in the image.
  double getXMin() const;

  /// Get the minimum y-coordinate in the image
  double getYMin() const;

  /// Get the maximum x-coordinate in the image.
  double getXMax() const;

  /// Get the maximum y-coordinate in the image
  double getYMax() const;

  /**
   * Initialize the scan-converter.
   */
  void init(unsigned int AN, unsigned int LN, double speedOfSound, double resolution,
		double radius, int inputHeight, double pitch, int nElements);

  void init(unsigned int AN, unsigned int LN, double speedOfSound, double resolution,
		double radius, double samplingFrequency, double pitch, int nElements);

  /**
   * Run the scan-converter.
   */
  void run(usImagePostScan2D<unsigned char> &Dst, const usImagePreScan2D<unsigned char> &Src);
  double interpolateLinear(const vpImage<unsigned char>& I, double x, double y);


  vpImage<unsigned char> getMask() const;

  bool isInFOV(double i, double j) const;

 private:
  usImagePostScan2D<unsigned char> m_postScanImage;

  unsigned int m_AN;
  unsigned int m_LN;
  double m_speedOfSound;
  bool m_curved;
  double m_radius;
  double m_APitch;
  double m_LPitch;
  double m_resolution;
  double m_x_min;
  double m_x_max;
  double m_y_min;
  double m_y_max;
  unsigned int m_height;
  unsigned int m_width;
  vpMatrix m_rMap;
  vpMatrix m_tMap;

};

#endif // US_SCAN_CONVERTER_2D_H
