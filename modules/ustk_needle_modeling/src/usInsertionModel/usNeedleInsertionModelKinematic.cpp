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

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelKinematic.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>

usNeedleInsertionModelKinematic::usNeedleInsertionModelKinematic() : m_needle(), m_naturalCurvature(0) {}

usNeedleInsertionModelKinematic::usNeedleInsertionModelKinematic(const usNeedleInsertionModelKinematic &needle)
  : m_needle(needle.m_needle), m_naturalCurvature(needle.m_naturalCurvature)
{
}

usNeedleInsertionModelKinematic::~usNeedleInsertionModelKinematic() {}

usNeedleInsertionModelKinematic &usNeedleInsertionModelKinematic::
operator=(const usNeedleInsertionModelKinematic &needle)
{
  m_needle = needle.m_needle;

  m_naturalCurvature = needle.m_naturalCurvature;

  return (*this);
}

usNeedleInsertionModelKinematic *usNeedleInsertionModelKinematic::clone() const
{
  return new usNeedleInsertionModelKinematic(*this);
}

void usNeedleInsertionModelKinematic::setNaturalCurvature(double naturalCurvature)
{
  m_naturalCurvature = naturalCurvature;
}

double usNeedleInsertionModelKinematic::getNaturalCurvature() const { return m_naturalCurvature; }

const usNeedleModelBaseTip &usNeedleInsertionModelKinematic::accessNeedle() const { return m_needle; }

usNeedleModelBaseTip &usNeedleInsertionModelKinematic::accessNeedle() { return m_needle; }

bool usNeedleInsertionModelKinematic::moveBase(double vz, double wz, double time)
{
  vpColVector command(6, 0);
  command[2] = vz;
  command[5] = wz;

  return this->usNeedleInsertionModelInterface::moveBase(command, time);
}

bool usNeedleInsertionModelKinematic::moveBase(double controlCurvature, double vz, double wz, double time)
{
  double naturalCurvature = m_naturalCurvature;
  m_naturalCurvature = controlCurvature;

  bool moved = this->moveBase(vz, wz, time);

  m_naturalCurvature = naturalCurvature;

  return moved;
}

bool usNeedleInsertionModelKinematic::setBasePose(const vpPoseVector &pose)
{
  vpHomogeneousMatrix H(pose);
  vpColVector baseVelocity = vpExponentialMap::inverse(m_needle.getWorldMbase().inverse() * H, 1);

  m_needle.setBasePose(pose);

  vpColVector tipVelocity(6, 0);
  tipVelocity[2] = baseVelocity[2];
  tipVelocity[3] = m_naturalCurvature * tipVelocity[2];
  tipVelocity[5] = baseVelocity[5];

  m_needle.moveTip(tipVelocity, 1);

  return true;
}

vpPoseVector usNeedleInsertionModelKinematic::getBasePose() const { return m_needle.getBasePose(); }
