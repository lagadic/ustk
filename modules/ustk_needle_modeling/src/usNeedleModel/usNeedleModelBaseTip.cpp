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

#include <visp3/ustk_needle_modeling/usNeedleModelBaseTip.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>

usNeedleModelBaseTip::usNeedleModelBaseTip() : m_basePose(), m_worldMbase(), m_tipPose(), m_worldMtip() {}

usNeedleModelBaseTip::usNeedleModelBaseTip(const usNeedleModelBaseTip &needle)
  : m_basePose(needle.m_basePose), m_worldMbase(needle.m_worldMbase), m_tipPose(needle.m_tipPose),
    m_worldMtip(needle.m_worldMtip)
{
}

usNeedleModelBaseTip::~usNeedleModelBaseTip() {}

usNeedleModelBaseTip &usNeedleModelBaseTip::operator=(const usNeedleModelBaseTip &needle)
{
  m_basePose = needle.m_basePose;
  m_worldMbase = needle.m_worldMbase;
  m_tipPose = needle.m_tipPose;
  m_worldMtip = needle.m_worldMtip;

  return (*this);
}

usNeedleModelBaseTip *usNeedleModelBaseTip::clone() const { return new usNeedleModelBaseTip(*this); }

vpPoseVector usNeedleModelBaseTip::getBasePose() const { return m_basePose; }

vpHomogeneousMatrix usNeedleModelBaseTip::getWorldMbase() const { return m_worldMbase; }

vpColVector usNeedleModelBaseTip::getBasePosition() const { return m_basePose.getTranslationVector(); }

vpColVector usNeedleModelBaseTip::getBaseDirection() const
{
  vpColVector v(3);

  v[0] = m_worldMbase[0][2];
  v[1] = m_worldMbase[1][2];
  v[2] = m_worldMbase[2][2];

  return v;
}

vpPoseVector usNeedleModelBaseTip::getTipPose() const { return m_tipPose; }
vpHomogeneousMatrix usNeedleModelBaseTip::getWorldMtip() const { return m_worldMtip; }

vpColVector usNeedleModelBaseTip::getTipPosition() const { return m_tipPose.getTranslationVector(); }

vpColVector usNeedleModelBaseTip::getTipDirection() const
{
  vpColVector v(3);

  v[0] = m_worldMtip[0][2];
  v[1] = m_worldMtip[1][2];
  v[2] = m_worldMtip[2][2];

  return v;
}

void usNeedleModelBaseTip::setBasePose(double tx, double ty, double tz, double thetax, double thetay, double thetaz)
{
  m_worldMbase.buildFrom(tx, ty, tz, thetax, thetay, thetaz);
  m_basePose[0] = tx;
  m_basePose[1] = ty;
  m_basePose[2] = tz;
  m_basePose[3] = thetax;
  m_basePose[4] = thetay;
  m_basePose[5] = thetaz;
}

void usNeedleModelBaseTip::setBasePose(const vpPoseVector &pose)
{
  m_basePose = pose;
  m_worldMbase.buildFrom(m_basePose);
}

void usNeedleModelBaseTip::setBasePose(const vpHomogeneousMatrix &Hpose)
{
  m_worldMbase = Hpose;
  m_basePose.buildFrom(m_worldMbase);
}

void usNeedleModelBaseTip::setTipPose(double tx, double ty, double tz, double thetax, double thetay, double thetaz)
{
  m_worldMtip.buildFrom(tx, ty, tz, thetax, thetay, thetaz);
  m_tipPose[0] = tx;
  m_tipPose[1] = ty;
  m_tipPose[2] = tz;
  m_tipPose[3] = thetax;
  m_tipPose[4] = thetay;
  m_tipPose[5] = thetaz;
}

void usNeedleModelBaseTip::setTipPose(const vpPoseVector &pose)
{
  m_tipPose = pose;
  m_worldMtip.buildFrom(m_tipPose);
}

void usNeedleModelBaseTip::setTipPose(const vpHomogeneousMatrix &Hpose)
{
  m_worldMtip = Hpose;
  m_tipPose.buildFrom(m_worldMtip);
}

void usNeedleModelBaseTip::moveBase(const vpColVector &control, double time)
{
  if (control.size() != 6)
    throw vpException(
        vpException::dimensionError,
        "usNeedleModelBaseTip::moveBase(const vpColVector&, double): invalid control vector dimension, should be 6");
  if (time <= std::numeric_limits<double>::epsilon())
    return;
  if (control.frobeniusNorm() <= std::numeric_limits<double>::epsilon())
    return;

  vpHomogeneousMatrix Hmotion = vpExponentialMap::direct(control, time);

  this->moveBase(Hmotion);
}

void usNeedleModelBaseTip::moveBase(double tx, double ty, double tz, double thetax, double thetay, double thetaz)
{
  vpHomogeneousMatrix Hmotion(tx, ty, tz, thetax, thetay, thetaz);

  this->moveBase(Hmotion);
}

void usNeedleModelBaseTip::moveBase(const vpPoseVector &pose)
{
  vpHomogeneousMatrix Hmotion(pose);

  this->moveBase(Hmotion);
}

void usNeedleModelBaseTip::moveBase(const vpColVector &v)
{
  if (v.size() != 6)
    throw vpException(vpException::dimensionError,
                      "usNeedleModelBaseTip::moveBase(const vpColVector): invalid vector dimension, should be 6");
  vpHomogeneousMatrix Hmotion(v[0], v[1], v[2], v[3], v[4], v[5]);

  this->moveBase(Hmotion);
}

void usNeedleModelBaseTip::moveBase(const vpHomogeneousMatrix &Hmotion)
{
  // Move base
  // Translation and rotation are expressed in base frame

  this->setBasePose(m_worldMbase * Hmotion);
}

void usNeedleModelBaseTip::moveBaseWorldFrame(const vpColVector &control, double time)
{
  if (control.size() != 6)
    throw vpException(vpException::dimensionError, "usNeedleModelBaseTip::usNeedleModelBaseWorldFrame(const "
                                                   "vpColVector&, double): invalid control vector dimension, should be "
                                                   "6");
  if (time <= std::numeric_limits<double>::epsilon())
    return;
  if (control.frobeniusNorm() <= std::numeric_limits<double>::epsilon())
    return;

  vpHomogeneousMatrix Hmotion = vpExponentialMap::direct(control, time);

  this->moveBaseWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveBaseWorldFrame(double tx, double ty, double tz, double thetax, double thetay,
                                              double thetaz)
{
  vpHomogeneousMatrix Hmotion(tx, ty, tz, thetax, thetay, thetaz);

  this->moveBaseWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveBaseWorldFrame(const vpPoseVector &pose)
{
  vpHomogeneousMatrix Hmotion(pose);

  this->moveBaseWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveBaseWorldFrame(const vpColVector &v)
{
  if (v.size() != 6)
    throw vpException(
        vpException::dimensionError,
        "usNeedleModelBaseTip::moveBaseWorldFrame(const vpColVector&): invalid vector dimension, should be 6");
  vpHomogeneousMatrix Hmotion(v[0], v[1], v[2], v[3], v[4], v[5]);

  this->moveBaseWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveBaseWorldFrame(const vpHomogeneousMatrix &Hmotion)
{
  // Move base
  // Translation is expressed in world frame,
  // Rotation is expressed in world frame but is done around the base position

  vpHomogeneousMatrix Hrotation0(m_worldMbase);

  Hrotation0[0][3] = 0;
  Hrotation0[1][3] = 0;
  Hrotation0[2][3] = 0;

  vpHomogeneousMatrix HmotionBaseFrame = Hrotation0.inverse() * Hmotion * Hrotation0;

  this->moveBase(HmotionBaseFrame);
}

void usNeedleModelBaseTip::moveTip(const vpColVector &control, double time)
{
  if (control.size() != 6)
    throw vpException(
        vpException::dimensionError,
        "usNeedleModelBaseTip::moveTip(const vpColVector&, double): invalid control vector dimension, should be 6");
  if (time <= std::numeric_limits<double>::epsilon())
    return;
  if (control.frobeniusNorm() <= std::numeric_limits<double>::epsilon())
    return;

  vpHomogeneousMatrix Hmotion = vpExponentialMap::direct(control, time);

  this->moveTip(Hmotion);
}

void usNeedleModelBaseTip::moveTip(double tx, double ty, double tz, double thetax, double thetay, double thetaz)
{
  vpHomogeneousMatrix Hmotion(tx, ty, tz, thetax, thetay, thetaz);

  return this->moveTip(Hmotion);
}

void usNeedleModelBaseTip::moveTip(const vpPoseVector &pose)
{
  vpHomogeneousMatrix Hmotion(pose);

  return this->moveTip(Hmotion);
}

void usNeedleModelBaseTip::moveTip(const vpColVector &v)
{
  if (v.size() != 6)
    throw vpException(vpException::dimensionError,
                      "usNeedleModelBaseTip::MoveTip(const vpColVector&): invalid vector dimension, should be 6");
  vpHomogeneousMatrix Hmotion(v[0], v[1], v[2], v[3], v[4], v[5]);

  return this->moveTip(Hmotion);
}

void usNeedleModelBaseTip::moveTip(const vpHomogeneousMatrix &Hmotion)
{
  // Move base
  // Translation and rotation are expressed in base frame

  return this->setTipPose(m_worldMtip * Hmotion);
}

void usNeedleModelBaseTip::moveTipWorldFrame(const vpColVector &control, double time)
{
  if (control.size() != 6)
    throw vpException(vpException::dimensionError, "usNeedleModelBaseTip::moveTipWorldFrame(const vpColVector&, "
                                                   "double): invalid control vector dimension, should be 6");
  if (time <= std::numeric_limits<double>::epsilon())
    return;
  if (control.frobeniusNorm() <= std::numeric_limits<double>::epsilon())
    return;

  vpHomogeneousMatrix Hmotion = vpExponentialMap::direct(control, time);

  this->moveTipWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveTipWorldFrame(double tx, double ty, double tz, double thetax, double thetay,
                                             double thetaz)
{
  vpHomogeneousMatrix Hmotion(tx, ty, tz, thetax, thetay, thetaz);

  this->moveTipWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveTipWorldFrame(const vpPoseVector &pose)
{
  vpHomogeneousMatrix Hmotion(pose);

  this->moveTipWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveTipWorldFrame(const vpColVector &v)
{
  if (v.size() != 6)
    throw vpException(
        vpException::dimensionError,
        "usNeedleModelBaseTip::moveTipWorldFrame(const vpColVector&): invalid vector dimension, should be 6");
  vpHomogeneousMatrix Hmotion(v[0], v[1], v[2], v[3], v[4], v[5]);

  this->moveTipWorldFrame(Hmotion);
}

void usNeedleModelBaseTip::moveTipWorldFrame(const vpHomogeneousMatrix &Hmotion)
{
  // Move tip
  // Translation is expressed in world frame,
  // Rotation is expressed in world frame but is done around the tip position

  vpHomogeneousMatrix Hrotation0(m_worldMtip);

  Hrotation0[0][3] = 0;
  Hrotation0[1][3] = 0;
  Hrotation0[2][3] = 0;

  vpHomogeneousMatrix HmotionBaseFrame = Hrotation0.inverse() * Hmotion * Hrotation0;

  this->moveTip(HmotionBaseFrame);
}

std::ostream &operator<<(std::ostream &s, const usNeedleModelBaseTip &needle)
{
  s << "usNeedleModelBaseTip\n";
  for (int i = 0; i < 6; i++)
    s << needle.m_basePose[i] << " ";
  s << '\n';
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++)
      s << needle.m_worldMbase[i][j] << " ";
    s << '\n';
  }
  for (int i = 0; i < 6; i++)
    s << needle.m_tipPose[i] << " ";
  s << '\n';
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++)
      s << needle.m_worldMtip[i][j] << " ";
    s << '\n';
  }

  s.flush();
  return s;
}

std::istream &operator>>(std::istream &s, usNeedleModelBaseTip &needle)
{
  std::string c;
  s >> c;
  if (c != "usNeedleModelBaseTip") {
    vpException e(vpException::ioError, "Stream does not contain usNeedleModelBaseTip data");
    throw e;
  }
  for (int i = 0; i < 6; i++)
    s >> needle.m_basePose[i];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      s >> needle.m_worldMbase[i][j];
  for (int i = 0; i < 6; i++)
    s >> needle.m_tipPose[i];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      s >> needle.m_worldMtip[i][j];
  return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleModelBaseTip &needle)
{
  s.write("usNeedleModelBaseTip", 21);
  for (int i = 0; i < 6; i++)
    s.write((char *)&(needle.m_basePose[i]), sizeof(double));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      s.write((char *)&(needle.m_worldMbase[i][j]), sizeof(double));
  for (int i = 0; i < 6; i++)
    s.write((char *)&(needle.m_tipPose[i]), sizeof(double));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      s.write((char *)&(needle.m_worldMtip[i][j]), sizeof(double));

  s.flush();
  return s;
}

std::istream &operator>>=(std::istream &s, usNeedleModelBaseTip &needle)
{
  char c[21];
  s.read(c, 21);
  if (strcmp(c, "usNeedleModelBaseTip")) {
    vpException e(vpException::ioError, "Stream does not contain usNeedleModelBaseTip data");
    throw e;
  }
  for (int i = 0; i < 6; i++)
    s.read((char *)&(needle.m_basePose[i]), sizeof(double));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      s.read((char *)&(needle.m_worldMbase[i][j]), sizeof(double));
  for (int i = 0; i < 6; i++)
    s.read((char *)&(needle.m_tipPose[i]), sizeof(double));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      s.read((char *)&(needle.m_worldMtip[i][j]), sizeof(double));
  return s;
}
