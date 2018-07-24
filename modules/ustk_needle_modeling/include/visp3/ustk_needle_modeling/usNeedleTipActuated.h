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
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

#ifndef __usNeedleTipActuated_h
#define __usNeedleTipActuated_h

#include <iostream>

#include <visp3/ustk_needle_modeling/usNeedleTip.h>

class VISP_EXPORT usNeedleTipActuated : public usNeedleTip
{
protected:
  double _diameter;
  double _length;
  double _angle;         // rad   angle between needle axis and tip direction
  double _steeringAngle; // rad   angle with respect to x axis

public:
  //! Constructors, destructors

  usNeedleTipActuated();
  usNeedleTipActuated(const usNeedleTipActuated &needle);
  virtual ~usNeedleTipActuated();
  virtual usNeedleTipActuated &operator=(const usNeedleTipActuated &needle);
  virtual usNeedleTipActuated *clone() const; // Polymorph copy method

  //! Parameters setters and getters

  void setDiameter(double diameter);
  double getDiameter() const;
  void setLength(double l);
  double getLength() const;
  void setTipAngleRad(double angle);
  double getTipAngleRad() const;
  void setTipAngleDeg(double angle);
  double getTipAngleDeg() const;
  void setSteeringAngleRad(double angle);
  double getSteeringAngleRad() const;
  void setSteeringAngleDeg(double angle);
  double getSteeringAngleDeg() const;

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTipActuated &tip);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTipActuated &tip);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTipActuated &tip);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTipActuated &tip);

private:
  void updateTipPose();
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTipActuated &tip);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTipActuated &tip);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTipActuated &tip);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTipActuated &tip);

#endif // __usNeedleTipActuated_h
