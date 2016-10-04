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
 * @brief Ultrasound image settings for 2D postscan images.
 */

#ifndef US_IMAGE_POSTSCAN_SETTINGS_H
#define US_IMAGE_POSTSCAN_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usImageSettings.h>

/**
 * @class usImagePostScanSettings
 * @brief Ultrasound image settings for 2D postscan images.
 */
class VISP_EXPORT usImagePostScanSettings : public usImageSettings {
public:
  usImagePostScanSettings();

  virtual ~usImagePostScanSettings();

  usImagePostScanSettings& operator=(const usImagePostScanSettings& other);

  bool operator==(const usImagePostScanSettings& other);

  void setWidthResolution(const double widthResolution);

  double getWidthResolution() const;

  void setHeightResolution(const double heightResolution);

  double getHeightResolution() const;

private:
  //Settings from the probe
  double m_widthResolution;
  double m_heightResolution;
};

#endif // US_IMAGE_POSTSCAN_SETTINGS_H
