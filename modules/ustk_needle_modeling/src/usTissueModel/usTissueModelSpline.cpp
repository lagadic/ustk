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

#include <visp3/ustk_needle_modeling/usTissueModelSpline.h>

usTissueModelSpline::usTissueModelSpline() : m_surface(), m_path() {}

usTissueModelSpline::usTissueModelSpline(const usTissueModelSpline &tissue)
  : m_surface(tissue.m_surface), m_path(tissue.m_path)
{
}

usTissueModelSpline::~usTissueModelSpline() {}

const usTissueModelSpline &usTissueModelSpline::operator=(const usTissueModelSpline &tissue)
{
  m_surface = tissue.m_surface;
  m_path = tissue.m_path;

  return *this;
}

usTissueModelSpline *usTissueModelSpline::clone() const { return new usTissueModelSpline(*this); }

const usOrientedPlane3D &usTissueModelSpline::accessSurface() const { return m_surface; }

usOrientedPlane3D &usTissueModelSpline::accessSurface() { return m_surface; }

const usBSpline3D &usTissueModelSpline::accessPath() const { return m_path; }

usBSpline3D &usTissueModelSpline::accessPath() { return m_path; }

bool usTissueModelSpline::moveInWorldFrame(const vpHomogeneousMatrix &H)
{
  m_surface.moveInWorldFrame(H);
  m_path.move(H);

  return true;
}

bool usTissueModelSpline::moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz)
{
  return this->moveInWorldFrame(vpHomogeneousMatrix(x, y, z, tx, ty, tz));
}

bool usTissueModelSpline::move(const vpHomogeneousMatrix &H)
{
  vpPoseVector ptu(this->getPose());

  vpHomogeneousMatrix M(ptu);
  M = M * H * M.inverse();
  m_surface.moveInWorldFrame(M);
  m_path.move(M);

  return true;
}

bool usTissueModelSpline::move(double x, double y, double z, double tx, double ty, double tz)
{
  return this->move(vpHomogeneousMatrix(x, y, z, tx, ty, tz));
}

bool usTissueModelSpline::setPose(const vpPoseVector &p)
{
  vpPoseVector ptu(this->getPose());

  vpHomogeneousMatrix M(vpHomogeneousMatrix(p) * vpHomogeneousMatrix(ptu).inverse());
  m_surface.moveInWorldFrame(M);
  m_path.move(M);

  return true;
}

vpPoseVector usTissueModelSpline::getPose() const
{
  vpPoseVector ptu = m_surface.getPose();

  if (m_path.getNbSegments() > 0) {
    vpColVector p = m_path.getPoint(0);
    for (int i = 0; i < 3; i++)
      ptu[i] = p[i];
  }

  return ptu;
}

std::ostream &operator<<(std::ostream &s, const usTissueModelSpline &tissue)
{
  s << "usTissueModelSpline\n";
  s << tissue.m_surface;
  s << tissue.m_path;

  s.flush();
  return s;
}

std::istream &operator>>(std::istream &s, usTissueModelSpline &tissue)
{
  std::string c;
  s >> c;
  if (c != "usTissueModelSpline") {
    vpException e(vpException::ioError, "Stream does not contain usTissueModelSpline data");
    throw e;
  }
  s >> tissue.m_surface;
  s >> tissue.m_path;
  return s;
}

std::ostream &operator<<=(std::ostream &s, const usTissueModelSpline &tissue)
{
  s.write("usTissueModelSpline", 20);
  s <<= tissue.m_surface;
  s <<= tissue.m_path;

  s.flush();
  return s;
}

std::istream &operator>>=(std::istream &s, usTissueModelSpline &tissue)
{
  char c[20];
  s.read(c, 20);
  if (strcmp(c, "usTissueModelSpline")) {
    vpException e(vpException::ioError, "Stream does not contain usTissueModelSpline data");
    throw e;
  }
  s >>= tissue.m_surface;
  s >>= tissue.m_path;
  return s;
}
