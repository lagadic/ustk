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
 * @file usImagePreScanSettings.h
 * @brief Generic ultrasound image settings.
 */

#ifndef US_IMAGE_PRESCAN_SETTINGS_H
#define US_IMAGE_PRESCAN_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include<visp3/ustk_core/usTransducerSettings.h>

/**
  @class usImagePreScanSettings
  @brief Settings associated to ultrasound pre-scan images implemented in usImageRF2D, usImageRF3D,
  usImagePreScan2D and usImagePreScan3D.
  @ingroup module_ustk_core

  This class represents ultrasound pre-scan image settings which are:
  - the common settings implemented in usTransducerSettings corresponding to the transducer settings.
    We recall that these common settings are:
    - the name of the probe that could be set using setProbeName() or retrieved using getProbeName().
    - the transducer radius \f$R_{_T}\f$ in meters (value set to zero for a linear transducer).
      Its value could be set using setTransducerRadius() and retrieved using getTransducerRadius().
    - the scan line pitch that corresponds to the angle \f$\alpha_{_{SC}}\f$ (in radians) between
      two successive scan line beams when the transducer is convex, or to the distance \f$d_{_{SC}}\f$
      (in meters) when the transducer is linear. To set this value use setScanLinePitch() and to get
      its value use getScanLinePitch().
    - the number of scan lines \f$n_{_{SC}}\f$. To set this setting use setScanLineNumber() and to access
      to the value use getScanLineNumber().
    - the type of ultrasound transducer used for data acquisition: convex or linear. This parameter
      could be set using setTransducerConvexity(). To know the transducer type use isTransducerConvex().
    - the depth that corresponds to the distance in meters between the first and the last pixel in a scan line.
      To set this value use setDepth() and to get the depth use getDepth().
    .
  - and an additional axial resolution parameter called \f$a_{_R}\f$ which corresponds to the
    distance (in meters) between two consecutive samples along a scan line. To set this value use
    setAxialResolution() and to retrieve this value use getAxialResolution(). Note that \f$a_{_R}\f$
    may differ for RF images and pre-scan images.

 */
class VISP_EXPORT usImagePreScanSettings : public usTransducerSettings {
public:
  usImagePreScanSettings();
  usImagePreScanSettings(const usTransducerSettings &transducerSettings, double axialResolution);
  usImagePreScanSettings(const usImagePreScanSettings &other);
  virtual ~usImagePreScanSettings();

  /** @name Inherited functionalities from usImagePreScanSettings */
  //@{

  double getAxialResolution() const;

  usImagePreScanSettings& operator=(const usImagePreScanSettings& other);
  bool operator==(const usImagePreScanSettings& other);

  void setAxialResolution(const double axialResolution);
  void setImagePreScanSettings(const usImagePreScanSettings& preScanSettings);

  //@}

private:
  //Settings from the probe
  double m_axialResolution;
};
#endif // US_IMAGE_PRESCAN_SETTINGS_H
