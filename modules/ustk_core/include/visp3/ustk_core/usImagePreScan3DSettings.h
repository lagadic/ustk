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
* @file usImagePreScan3DSettings.h
* @brief Ultrasound image settings for 3D postscan images.
*/

#ifndef US_IMAGE_POSTSCAN_3D_SETTINGS_H
#define US_IMAGE_POSTSCAN_3D_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usImageSettings3D.h>
/**
* @class usImagePreScan3DSettings
* @brief Ultrasound image settings for 3D postscan images.
*/
class VISP_EXPORT usImagePreScan3DSettings : public usImageSettings3D {
public:
  usImagePreScan3DSettings();

  virtual ~usImagePreScan3DSettings();

  usImagePreScan3DSettings& operator=(const usImagePreScan3DSettings& other);

  bool operator==(const usImagePreScan3DSettings& other);

  void setAxialResolution(const double axialResolution);

  double getAxialResolution() const;

private:
  //Settings from the probe
  double m_axialResolution;
};

#endif // US_IMAGE_POSTSCAN_3D_SETTINGS_H
