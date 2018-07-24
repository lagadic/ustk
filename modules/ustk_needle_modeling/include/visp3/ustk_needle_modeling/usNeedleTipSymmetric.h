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

#ifndef __usNeedleTipSymmetric_h
#define __usNeedleTipSymmetric_h

#include <iostream>

#include <visp3/ustk_needle_modeling/usNeedleTip.h>

class VISP_EXPORT usNeedleTipSymmetric : public usNeedleTip
{
protected:
  double _diameter;
  double _length;

public:
  //! Constructors, destructors

  usNeedleTipSymmetric();
  usNeedleTipSymmetric(const usNeedleTipSymmetric &needle);
  virtual ~usNeedleTipSymmetric();
  virtual usNeedleTipSymmetric &operator=(const usNeedleTipSymmetric &needle);
  virtual usNeedleTipSymmetric *clone() const; // Polymorph copy method

  //! Parameters setters and getters

  void setDiameter(double diameter);
  double getDiameter() const;
  void setLength(double l);
  double getLength() const;
  double getAngle() const; // rad

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTipSymmetric &tip);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTipSymmetric &tip);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTipSymmetric &tip);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTipSymmetric &tip);

private:
  void updateTipPose();
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTipSymmetric &tip);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTipSymmetric &tip);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTipSymmetric &tip);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTipSymmetric &tip);

#endif // __usNeedleTipSymmetric_h
