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
 * @class usImagePreScanSettings
 * @brief Settings associated to ultrasound pre-scan images.
 *
 * This class represents ultrasound pre-scan image settings which are:
 * - the common settings implemented in usImageSettings corresponding to the transducer settings.
 *   We recall that these common settings are:
 *   - the type of ultrasound transducer used for data acquisition: convex or linear
 *   - the transducer radius in meters (value set to zero for a linear transducer)
 *   - the scan line pitch that corresponds to the angle (in radians) between
 *     to successive scan lines beams when the transducer is convex, or to the distance (in meters)
 *     when the transducer is linear. See usImageSettings description for more details.
 *   .
 * - the image axial resolution which corresponds to the size (in meters) of a pixel along the
 *   scan line beam.
 */
class VISP_EXPORT usImagePreScanSettings : public usImageSettings {
public:
  usImagePreScanSettings();
  usImagePreScanSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double axial_resolution);
  virtual ~usImagePreScanSettings();

  double getAxialResolution() const;

  usImagePreScanSettings& operator=(const usImagePreScanSettings& other);
  bool operator==(const usImagePreScanSettings& other);

  void setAxialResolution(const double axialResolution);

private:
  //Settings from the probe
  double m_axialResolution;
};
#endif // US_IMAGE_PRESCAN_SETTINGS_H
