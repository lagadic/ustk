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
 * @brief Ultrasound image settings for 3D post-scan images.
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
 *
 * This class represents 3D ultrasound post-scan image settings which are:
 * - the common settings corresponding to the transducer and motor settings.
 *   See usImage3DSettings for more details.
 * - the image with and height resolution which corresponds to the size (in meters) of a pixel
 *   in the image.
 */
class VISP_EXPORT usImagePostScan3DSettings : public usImage3DSettings {
public:
  usImagePostScan3DSettings();
  usImagePostScan3DSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, double motorRadius, double framePitch, bool isMotorRotating, double height_resolution, double width_resolution);
  virtual ~usImagePostScan3DSettings();

  double getHeightResolution() const;
  double getWidthResolution() const;

  usImagePostScan3DSettings& operator=(const usImagePostScan3DSettings& other);
  bool operator==(const usImagePostScan3DSettings& other);

  void setHeightResolution(const double heightResolution);
  void setWidthResolution(const double widthResolution);

private:
  //Settings from the probe
  double m_widthResolution;
  double m_heightResolution;
};

/**
* Operator to print image informations on a stream.
*/
std::ostream& operator<<(std::ostream& out, const usImagePostScan3DSettings &other)
{
  out << static_cast<const usImage3DSettings &>(other);
  return out << "Height resolution : " << other.getHeightResolution() <<
         "Width resolution : " << other.getWidthResolution();
}

#endif // US_IMAGE_POSTSCAN_3D_SETTINGS_H
