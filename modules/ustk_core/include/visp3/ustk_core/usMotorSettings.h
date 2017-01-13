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
 *
 *****************************************************************************/

/**
 * @file usMotorSettings.h
 * @brief Generic ultrasound probe motor settings.
 */

#ifndef __usMotorSettings_h_
#define __usMotorSettings_h_

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include <visp3/ustk_core/usTransducerSettings.h>

/**
   @class usMotorSettings

   @brief Generic class for 3D ultrasound motor settings associated to the 3D probe used
   during acquisition.
   @ingroup module_ustk_core

   This class represents motor settings for 3D ultrasound images which are the common settings
   corresponding to the motor used to produce the third dimension.

   These settings are:
    - the type of motor used to move the transducer: linear, tilting (small rotation) or rotationnal
      (360&deg; rotation). This type is defined in usMotorType and could be set using setMotorType(). To retrieve
      the motor type use getMotorType().
    - the motor radius \f$R_{_M}\f$ (value set to zero for a linear motor). This value could be set using
      setMotorRadius() or get using getMotorRadius().
    - the frame pitch that corresponds to the angle \f$\alpha_{_F}\f$ (in radians) between
      to successive frame acquisitions when the motor is convex, or to the distance \f$d_{_F}\f$ (in meters)
      when the motor is linear. To set this value use setFramePitch() and to access use getFramePitch().
    - the frame number \f$n_{_F}\f$ that corresponds to the number of frames acquired by the probe to
      generate the 3D volume. This number is set using setFrameNumber() and could be retrieved using
      getFrameNumber().

   The following figure summerize these motor settings.

   \image html img-usMotorSettings.png
 */
class VISP_EXPORT usMotorSettings {
public:
  /*! Enumerator for motor type*/
  typedef enum {
    LinearMotor = 0,/*!< Case of a linear motor. */
    TiltingMotor,/*!< Case of a tilting motor (small rotation). */
    RotationalMotor/*!< Case of a roatational motor (360&deg; rotation). */
  } usMotorType;

  usMotorSettings();
  usMotorSettings(double motorRadius, double framePitch, unsigned int frameNumber, const usMotorType &motorType);
  usMotorSettings(const usMotorSettings &other);
  virtual ~usMotorSettings();

  /** @name Inherited functionalities from usMotorSettings */
  //@{
  unsigned int getFrameNumber() const;
  double getFramePitch() const;
  double getMotorRadius() const;
  usMotorType getMotorType() const;

  usMotorSettings& operator=(const usMotorSettings& other);
  bool operator==(const usMotorSettings& other);

  // Settings from the 3D probe
  void setFrameNumber(unsigned int frameNumber);
  void setFramePitch(double framePitch);
  void setMotorRadius(double motorRadius);
  void setMotorSettings(const usMotorSettings &other);
  void setMotorType(const usMotorType &motorType);

  //@}

  friend VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usMotorSettings& other);

private:
  //Settings from the 3D probe
  double m_motorRadius;
  double m_framePitch;
  unsigned int m_frameNumber;
  usMotorType m_motorType;
};

#endif // US_MOTOR_SETTINGS_H
