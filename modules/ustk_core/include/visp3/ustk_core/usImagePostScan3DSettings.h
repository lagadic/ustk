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
 * @file usImagePostScan3DSettings.h
 * @brief Ultrasound image settings for 3D postscan images.
 */

#ifndef US_IMAGE_POSTSCAN_3D_SETTINGS_H
#define US_IMAGE_POSTSCAN_3D_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usImage3DSettings.h>
/**
 * @class usImagePostScan3DSettings
 * @brief Ultrasound image settings for 3D postscan images.
 */
class VISP_EXPORT usImagePostScan3DSettings : public usImage3DSettings {
public:
  usImagePostScan3DSettings();

  usImagePostScan3DSettings(double probeRadius, double scanLinePitch, bool isProbeConvex, double motorRadius, double framePitch, bool isMotorConvex, double height_resolution, double width_resolution);

  virtual ~usImagePostScan3DSettings();

  usImagePostScan3DSettings& operator=(const usImagePostScan3DSettings& other);

  bool operator==(const usImagePostScan3DSettings& other);

  void setWidthResolution(const double widthResolution);

  double getWidthResolution() const;

  void setHeightResolution(const double heightResolution);

  double getHeightResolution() const;

private:
  //Settings from the probe
  double m_widthResolution;
  double m_heightResolution;
};

#endif // US_IMAGE_POSTSCAN_3D_SETTINGS_H
