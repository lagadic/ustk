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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImagePostScanSettings.h
 * @brief Ultrasound image settings for 2D post-scan images.
 */

#ifndef US_IMAGE_POSTSCAN_SETTINGS_H
#define US_IMAGE_POSTSCAN_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usTransducerSettings.h>

/**
 * @class usImagePostScanSettings
 * @brief Settings associated to ultrasound post-scan images.
 * @ingroup module_ustk_core
 *
 * This class represents ultrasound pre-scan image settings which are:
 * - the common settings implemented in usTransducerSettings for all ultrasound images.
 *   We recall that these common settings are:
 *   - the type of ultrasound transducer used for data acquisition: convex or linear
 *   - the transducer radius in meters (value set to zero for a linear transducer)
 *   - the scan line pitch that corresponds to the angle (in radians) between
 *     to succesive scan lines beams when the transducer is convex, or to the distance (in meters)
 *     when the transducer is linear. See usTransducerSettings description for more details.
 *   .
 * - the image with and height resolution which corresponds to the size (in meters) of a pixel
 *   in the image.
 */
class VISP_EXPORT usImagePostScanSettings : public usTransducerSettings {
public:
  usImagePostScanSettings();
  usImagePostScanSettings(const usImagePostScanSettings &other);
  usImagePostScanSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double height_resolution, double width_resolution);
  usImagePostScanSettings(const usTransducerSettings basicSettings, double height_resolution, double width_resolution);
  virtual ~usImagePostScanSettings();

  double getHeightResolution() const;
  double getWidthResolution() const;
  usImagePostScanSettings getImageSettings() const;

  usImagePostScanSettings& operator=(const usImagePostScanSettings& other);
  friend VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usTransducerSettings &other);
  bool operator==(const usImagePostScanSettings& other);

  void setHeightResolution(const double heightResolution);
  void setWidthResolution(const double widthResolution);
  void setImageSettings(const usImagePostScanSettings& postScanSettings);

private:
  //Settings from the probe
  double m_widthResolution;
  double m_heightResolution;
};

#endif // US_IMAGE_POSTSCAN_SETTINGS_H
