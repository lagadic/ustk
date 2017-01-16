/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#ifndef __usTransducerSettings_h_
#define __usTransducerSettings_h_

//std includes
#include <iostream>
#include <string>

//visp includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

//ustk includes

/*!
   @class usTransducerSettings
   @brief Generic class for 2D ultrasound data common settings associated to the type of probe transducer used during acquisition.
   @ingroup module_ustk_core

   This class represents ultrasound transducer common settings which are:
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

   Knowing the scan line pitch (\f$\alpha_{_{SC}}\f$ or \f$d_{_{SC}}\f$) and the number of scan lines
   \f$n_{_{SC}}\f$, you can get the transducer field of view using getFieldOfView().

   The following figure summerize these transducer settings.

   \image html img-usTransducerSettings.png
 */
class VISP_EXPORT usTransducerSettings {
public:
  usTransducerSettings();
  usTransducerSettings(double transducerRadius, double scanLinePitch, unsigned int scanLineNumber, bool transducerConvex, double depth);
  usTransducerSettings(const usTransducerSettings &other);

  virtual ~usTransducerSettings();

  /** @name Inherited functionalities from usTransducerSettings */
  //@{

  double getDepth() const;
  double getFieldOfView() const;
  std::string getProbeName() const;
  unsigned int getScanLineNumber() const;
  double getScanLinePitch() const;
  double getTransducerRadius() const;

  bool isTransducerConvex() const;

  usTransducerSettings& operator=(const usTransducerSettings& other);
  bool operator==(usTransducerSettings const& other);

  bool scanLineNumberIsSet() const;
  //Settings for the probe transducer
  void setDepth(double depth);
  void setFieldOfView(double fieldOfView);
  void setProbeName(std::string probeName);
  void setScanLineNumber(unsigned int scanLineNumber);
  void setScanLinePitch(const double scanLinePitch);
  void setTransducerConvexity(const bool isTransducerConvex);
  void setTransducerSettings(const usTransducerSettings& other);
  void setTransducerRadius(const double transducerRadius);

  //@}

  friend VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usTransducerSettings &other);

private:
  //Settings for the probe transducer
  double m_transducerRadius;
  double m_scanLinePitch;
  unsigned int m_scanLineNumber;
  bool m_isTransducerConvex;
  double m_depth;
  std::string m_probeName;

  bool m_scanLineNumberIsSet;
};

#endif // US_TRANSDUCER_SETTINGS_H
