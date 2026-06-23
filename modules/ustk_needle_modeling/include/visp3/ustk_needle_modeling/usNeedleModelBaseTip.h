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

#ifndef __usNeedleModelBaseTip_h_
#define __usNeedleModelBaseTip_h_

#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>

class VISP_EXPORT usNeedleModelBaseTip
{
protected:
  VISP_NAMESPACE_ADDRESSING vpPoseVector m_basePose;
  VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix m_worldMbase;

  VISP_NAMESPACE_ADDRESSING vpPoseVector m_tipPose;
  VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix m_worldMtip;

public:
  //! Constructors, destructors

  usNeedleModelBaseTip();
  usNeedleModelBaseTip(const usNeedleModelBaseTip &needle);
  virtual ~usNeedleModelBaseTip();
  virtual usNeedleModelBaseTip &operator=(const usNeedleModelBaseTip &needle);

  virtual usNeedleModelBaseTip *clone() const; // Polymorph copy method

  //! Parameters setters and getters

  VISP_NAMESPACE_ADDRESSING vpPoseVector getBasePose() const;
  VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix getWorldMbase() const;
  VISP_NAMESPACE_ADDRESSING vpColVector getBasePosition() const;
  VISP_NAMESPACE_ADDRESSING vpColVector getBaseDirection() const;

  VISP_NAMESPACE_ADDRESSING vpPoseVector getTipPose() const;
  VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix getWorldMtip() const;
  VISP_NAMESPACE_ADDRESSING vpColVector getTipPosition() const;
  VISP_NAMESPACE_ADDRESSING vpColVector getTipDirection() const;

  //! Command of the needle

  void setBasePose(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  void setBasePose(const VISP_NAMESPACE_ADDRESSING vpPoseVector &pose);
  void setBasePose(const VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix &Hpose);

  void setTipPose(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  void setTipPose(const VISP_NAMESPACE_ADDRESSING vpPoseVector &pose);
  void setTipPose(const VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix &Hpose);

  void moveBase(const VISP_NAMESPACE_ADDRESSING vpColVector &control, double time);
  void moveBase(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  void moveBase(const VISP_NAMESPACE_ADDRESSING vpPoseVector &pose);
  void moveBase(const VISP_NAMESPACE_ADDRESSING vpColVector &v);
  void moveBase(const VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix &Hmotion);

  void moveBaseWorldFrame(const VISP_NAMESPACE_ADDRESSING vpColVector &command, double time);
  void moveBaseWorldFrame(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  void moveBaseWorldFrame(const VISP_NAMESPACE_ADDRESSING vpPoseVector &pose);
  void moveBaseWorldFrame(const VISP_NAMESPACE_ADDRESSING vpColVector &v);
  void moveBaseWorldFrame(const VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix &Hmotion);

  void moveTip(const VISP_NAMESPACE_ADDRESSING vpColVector &control, double time);
  void moveTip(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  void moveTip(const VISP_NAMESPACE_ADDRESSING vpPoseVector &pose);
  void moveTip(const VISP_NAMESPACE_ADDRESSING vpColVector &v);
  void moveTip(const VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix &Hmotion);

  void moveTipWorldFrame(const VISP_NAMESPACE_ADDRESSING vpColVector &command, double time);
  void moveTipWorldFrame(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  void moveTipWorldFrame(const VISP_NAMESPACE_ADDRESSING vpPoseVector &pose);
  void moveTipWorldFrame(const VISP_NAMESPACE_ADDRESSING vpColVector &v);
  void moveTipWorldFrame(const VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix &Hmotion);

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleModelBaseTip &needle);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleModelBaseTip &needle);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleModelBaseTip &needle);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleModelBaseTip &needle);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleModelBaseTip &needle);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleModelBaseTip &needle);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleModelBaseTip &needle);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleModelBaseTip &needle);

#endif // usNeedleModelBaseTip_h
