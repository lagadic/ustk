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

#ifndef __usNeedleModelPolynomial_h_
#define __usNeedleModelPolynomial_h_

#include <iostream>

#include <visp3/core/vpColVector.h>

#include <visp3/ustk_core/usPolynomialCurve3D.h>
#include <visp3/ustk_needle_modeling/usNeedleModelBaseTip.h>

class VISP_EXPORT usNeedleModelPolynomial : public usNeedleModelBaseTip, public usPolynomialCurve3D
{
public:
  enum class NeedlePreset : int
  {
    BiopsyNeedle,
    BiopsyCannula,
    AbayazidRRM13,
    MisraRSRO10_PlastisolA,
    RoesthuisAM12,
    SteelSoftTissue
  };

protected:
  //! Needle parameters

  double m_outerDiameter;
  double m_insideDiameter;
  double m_needleYoungModulus;

public:
  using usNeedleModelBaseTip::operator=;
  //! Constructors, destructor

  usNeedleModelPolynomial();
  usNeedleModelPolynomial(const usNeedleModelPolynomial &needle);
  virtual ~usNeedleModelPolynomial();
  const usNeedleModelPolynomial &operator=(const usNeedleModelPolynomial &needle);

  virtual usNeedleModelPolynomial *clone() const { return new usNeedleModelPolynomial(*this); } // Polymorph copy method

  //! Parameters saving and loading

  void loadPreset(const NeedlePreset preset);

  //! Parameters setters and getters

  //! Needle parameters

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

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleModelPolynomial &needle);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleModelPolynomial &needle);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleModelPolynomial &needle);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleModelPolynomial &needle);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleModelPolynomial &needle);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleModelPolynomial &needle);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleModelPolynomial &needle);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleModelPolynomial &needle);

#endif // __usNeedleModelPolynomial_h
