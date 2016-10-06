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
* @brief Ultrasound image settings for 3D post-scan images.
*/

#ifndef US_IMAGE_PRESCAN_3D_SETTINGS_H
#define US_IMAGE_PRESCAN_3D_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usImage3DSettings.h>
/**
* @class usImagePreScan3DSettings
 * @brief Settings associated to 3D ultrasound pre-scan images.
 *
 * This class represents 3D ultrasound pre-scan image settings which are:
 * - the common settings corresponding to the transducer and motor settings.
 *   See usImage3DSettings for more details.
 * - the image axial resolution which corresponds to the size (in meters) of a pixel along the
 *   scan line beam.
*/
class VISP_EXPORT usImagePreScan3DSettings : public usImage3DSettings {
public:
  usImagePreScan3DSettings();
  usImagePreScan3DSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double motorRadius, double framePitch, bool isMotorRotating, double axial_resolution);
  virtual ~usImagePreScan3DSettings();

  double getAxialResolution() const;

  usImagePreScan3DSettings& operator=(const usImagePreScan3DSettings& other);
  bool operator==(const usImagePreScan3DSettings& other);

  void setAxialResolution(const double axialResolution);

private:
  //Settings from the probe
  double m_axialResolution;
};

#endif // US_IMAGE_PRESCAN_3D_SETTINGS_H
