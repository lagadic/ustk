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

#ifndef __usNeedleTipPrebent_h
#define __usNeedleTipPrebent_h

#include <iostream>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/ustk_needle_modeling/usNeedleTip.h>

class VISP_EXPORT usNeedleTipPrebent : public usNeedleTip
{
protected:
  double _diameter;
  double _length;
  double _angle; // rad

public:
  using usNeedleTip::operator=;
  //! Constructors, destructors

  usNeedleTipPrebent();
  usNeedleTipPrebent(const usNeedleTipPrebent &needle);
  virtual ~usNeedleTipPrebent();
  usNeedleTipPrebent &operator=(const usNeedleTipPrebent &needle);
  virtual usNeedleTipPrebent *clone() const; // Polymorph copy method

  //! Parameters setters and getters

  void setDiameter(double diameter);
  double getDiameter() const;
  void setLength(double l);
  double getLength() const;
  void setAngleRad(double angle);
  double getAngleRad() const;
  void setAngleDeg(double angle);
  double getAngleDeg() const;

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTipPrebent &tip);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTipPrebent &tip);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTipPrebent &tip);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTipPrebent &tip);

private:
  void updateTipPose();
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTipPrebent &tip);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTipPrebent &tip);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTipPrebent &tip);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTipPrebent &tip);

#endif // __usNeedleTipPrebent_h
