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

#ifndef __usTissueModelPolynomial_h
#define __usTissueModelPolynomial_h

#include <iostream>

#include <visp3/ustk_core/usOrientedPlane3D.h>
#include <visp3/ustk_core/usPolynomialCurve3D.h>

class VISP_EXPORT usTissueModelPolynomial
{
public:
  //! Tissue parameters

  usOrientedPlane3D m_surface;
  usPolynomialCurve3D m_path;

public:
  //! Constructors, destructor

  usTissueModelPolynomial();
  usTissueModelPolynomial(const usTissueModelPolynomial &tissue);
  virtual ~usTissueModelPolynomial();
  const usTissueModelPolynomial &operator=(const usTissueModelPolynomial &tissue);

  virtual usTissueModelPolynomial *clone() const; // Polymorph copy method

  //! Parameters setters and getters

  const usOrientedPlane3D &accessSurface() const;
  usOrientedPlane3D &accessSurface();
  const usPolynomialCurve3D &accessPath() const;
  usPolynomialCurve3D accessPath();

  bool moveInWorldFrame(const vpHomogeneousMatrix &H);
  bool moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz);
  bool move(const vpHomogeneousMatrix &H);
  bool move(double x, double y, double z, double tx, double ty, double tz);
  bool setPose(const vpPoseVector &p);
  vpPoseVector getPose() const;

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usTissueModelPolynomial &tissue);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usTissueModelPolynomial &tissue);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usTissueModelPolynomial &tissue);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usTissueModelPolynomial &tissue);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usTissueModelPolynomial &tissue);
VISP_EXPORT std::istream &operator>>(std::istream &s, usTissueModelPolynomial &tissue);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usTissueModelPolynomial &tissue);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usTissueModelPolynomial &tissue);

#endif // __usTissueModelPolynomial_h
