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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImageSettings.h
 * @brief Generic ultrasound image settings.
 */

#ifndef US_IMAGE_PRESCAN_SETTINGS_H
#define US_IMAGE_PRESCAN_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include<visp3/ustk_core/usImageSettings.h>

/**
 * @class usImageSettings
 * @brief Generic class for ultrasound data : storage of probe and scanner settings.
 *
 * This class represents a ultrasound image settings.
 */
class VISP_EXPORT usImagePreScanSettings : public usImageSettings {
public:
  usImagePreScanSettings();

  virtual ~usImagePreScanSettings();

  usImagePreScanSettings& operator=(const usImagePreScanSettings& other);

  bool operator==(const usImagePreScanSettings& other);

  void setAxialResolution(const double axialResolution);

  double getAxialResolution() const;

private:
  //Settings from the probe
  double m_axialResolution;
};

#endif // US_IMAGE_PRESCAN_SETTINGS_H
