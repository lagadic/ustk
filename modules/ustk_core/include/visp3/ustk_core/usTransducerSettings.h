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
 * @file usTransducerSettings.h
 * @brief Generic ultrasound transducer settings.
 */

#ifndef US_TRANSDUCER_SETTINGS_H
#define US_TRANSDUCER_SETTINGS_H

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes

/**
 * @class usTransducerSettings
 * @brief Generic class for 2D ultrasound data common settings associated to the type of probe used during acquisition.
 * @ingroup module_ustk_core
 *
 * This class represents ultrasound image common settings which are:
 * - the type of ultrasound transducer used for data acquisition: convex or linear
 * - the transducer radius \f$R\f$ in meters (value set to zero for a linear transducer)
 * - the scan line pitch that corresponds to the angle \f$\alpha\f$ (in radians) between
 *   to successive scan lines beams when the transducer is convex, or to the distance \f$d\f$
 *   (in meters) when the transducer is linear.
 *
 * \image html img-usProbeSettings.png
 */
class VISP_EXPORT usTransducerSettings {
public:
  usTransducerSettings();
  usTransducerSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex);
  usTransducerSettings(const usTransducerSettings &other);

  virtual ~usTransducerSettings();

  /** @name Inherited functionalities from usTransducerSettings */
  //@{

  double getProbeRadius() const;
  double getScanLinePitch() const;

  bool isTransducerConvex() const;

  usTransducerSettings& operator=(const usTransducerSettings& other);
  bool operator==(usTransducerSettings const& other);
  friend VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usTransducerSettings &other);

  //Settings form the probe
  void setTransducerSettings(const usTransducerSettings& other);
  void setTransducerConvexity(const bool isTransducerConvex);
  void setProbeRadius(const double probeRadius);
  void setScanLinePitch(const double scanLinePitch);  

  //@}

private:
  //Settings from the probe
  double m_probeRadius;
  double m_scanLinePitch;
  bool m_isTransducerConvex;
};

#endif // US_TRANSDUCER_SETTINGS_H
