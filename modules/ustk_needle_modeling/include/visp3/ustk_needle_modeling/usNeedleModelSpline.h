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

#ifndef __usNeedleModelSpline_h_
#define __usNeedleModelSpline_h_

#include <iostream>

#include <visp3/core/vpColVector.h>

#include <visp3/ustk_core/usBSpline3D.h>
#include <visp3/ustk_needle_modeling/usNeedleModelBaseTip.h>

class VISP_EXPORT usNeedleModelSpline : public usNeedleModelBaseTip, public usBSpline3D
{
public:
  enum class NeedlePreset : int {
    BiopsyNeedle,
    BiopsyCannula,
    Symmetric,
    AbayazidRRM13,
    MisraRSRO10_PlastisolA,
    RoesthuisAM12,
    SteelSoftTissue,
    SRL_ActuatedFBG,
    SRL_BiopsySimple,
    SRL_BiopsyNID,
    NDI_Pink_Stylet
  };

protected:
  //! Needle parameters

  double m_length;
  double m_outerDiameter;
  double m_insideDiameter;
  double m_needleYoungModulus;

public:
  //! Constructors, destructor

  usNeedleModelSpline();
  usNeedleModelSpline(const usNeedleModelSpline &needle);
  virtual ~usNeedleModelSpline();
  const usNeedleModelSpline &operator=(const usNeedleModelSpline &needle);

  virtual usNeedleModelSpline *clone() const { return new usNeedleModelSpline(*this); } // Polymorph copy method

  //! Parameters saving and loading

  void loadPreset(const NeedlePreset preset);

  //! Parameters setters and getters

  //! Needle parameters

  void setFullLength(double length);
  double getFullLength() const;

  void setOuterDiameter(double diameter);
  double getOuterDiameter() const;

  void setInsideDiameter(double diameter);
  double getInsideDiameter() const;

  void setNeedleYoungModulus(double E);
  double getNeedleYoungModulus() const;

  double getEI() const;

  //! Spline definition

  void init();

  //! Measure model information

  //! Needle position

  vpColVector getNeedlePoint(double l) const;
  vpColVector getNeedleDirection(double l) const;

  double getDistanceFromPoint(const vpColVector &P, double start = 0, double stop = -1, double threshold = 1e-5) const;

  //! Needle bending

  double getBendingEnergy() const;

  //! Force at base

  vpColVector getBaseStaticTorsor() const;

  //! Curvature

  double getCurvatureFromNeedleShape(double start, double end, vpColVector &center3D, vpColVector &direction3D) const;

  //! Display

  void showNeedlePoints() const;
  void showNeedleDirections() const;
  void showNeedleSegmentCoef() const;
  void showNeedleSegmentLength() const;

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleModelSpline &needle);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleModelSpline &needle);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleModelSpline &needle);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleModelSpline &needle);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleModelSpline &needle);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleModelSpline &needle);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleModelSpline &needle);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleModelSpline &needle);

#endif // __usNeedleModelSpline_h_
