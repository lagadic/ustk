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

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelInterface.h>

#include <limits>

#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>

bool usNeedleInsertionModelInterface::setBasePose(double tx, double ty, double tz, double thetax, double thetay,
                                                  double thetaz)
{
  return this->setBasePose(vpPoseVector(tx, ty, tz, thetax, thetay, thetaz));
}

bool usNeedleInsertionModelInterface::setBasePose(const vpHomogeneousMatrix &Hpose)
{
  return this->setBasePose(vpPoseVector(Hpose));
}

vpColVector usNeedleInsertionModelInterface::getBasePosition() const
{
  return vpColVector(this->getBasePose().getTranslationVector());
}

vpHomogeneousMatrix usNeedleInsertionModelInterface::getWorldMbase() const
{
  return vpHomogeneousMatrix(this->getBasePose());
}

bool usNeedleInsertionModelInterface::moveBase(const vpColVector &v, double time)
{
  if (v.size() != 6)
    throw vpException(vpException::dimensionError,
                      "usNeedleInsertionModelInterface::moveBase(const vpColVector&): invalid vector dimension");
  if (time < 0)
    throw vpException(vpException::badValue,
                      "usNeedleInsertionModelInterface::moveBase(const vpColVector&): negative time period");

  if (time <= std::numeric_limits<double>::epsilon())
    return true;
  if (v.euclideanNorm() <= std::numeric_limits<double>::epsilon())
    return true;

  return this->moveBase(vpExponentialMap::direct(v, time));
}

bool usNeedleInsertionModelInterface::moveBase(double tx, double ty, double tz, double thetax, double thetay,
                                               double thetaz)
{
  return this->moveBase(vpHomogeneousMatrix(tx, ty, tz, thetax, thetay, thetaz));
}

bool usNeedleInsertionModelInterface::moveBase(const vpPoseVector &pose)
{
  return this->moveBase(vpHomogeneousMatrix(pose));
}

bool usNeedleInsertionModelInterface::moveBase(const vpColVector &v)
{
  if (v.size() != 6)
    throw vpException(vpException::dimensionError,
                      "usNeedleInsertionModelInterface::moveBase(const vpColVector&): invalid vector dimension");

  return this->moveBase(vpHomogeneousMatrix(v[0], v[1], v[2], v[3], v[4], v[5]));
}

bool usNeedleInsertionModelInterface::moveBase(const vpHomogeneousMatrix &Hmotion)
{
  // Move base
  // Translation and rotation are expressed in base frame

  return this->setBasePose(this->getWorldMbase() * Hmotion);
}

bool usNeedleInsertionModelInterface::moveBaseWorldFrame(const vpColVector &v, double time)
{
  if (v.size() != 6)
    throw vpException(
        vpException::dimensionError,
        "usNeedleInsertionModelInterface::moveBaseWorldFrame(const vpColVector&): invalid vector dimension");

  if (time <= std::numeric_limits<double>::epsilon())
    return false;
  if (v.euclideanNorm() <= std::numeric_limits<double>::epsilon())
    return false;

  return this->moveBaseWorldFrame(vpExponentialMap::direct(v, time));
}

bool usNeedleInsertionModelInterface::moveBaseWorldFrame(double tx, double ty, double tz, double thetax, double thetay,
                                                         double thetaz)
{
  return this->moveBaseWorldFrame(vpHomogeneousMatrix(tx, ty, tz, thetax, thetay, thetaz));
}

bool usNeedleInsertionModelInterface::moveBaseWorldFrame(const vpPoseVector &pose)
{
  return this->moveBaseWorldFrame(vpHomogeneousMatrix(pose));
}

bool usNeedleInsertionModelInterface::moveBaseWorldFrame(const vpColVector &v)
{
  if (v.size() != 6)
    throw vpException(
        vpException::dimensionError,
        "usNeedleInsertionModelInterface::moveBaseWorldFrame(const vpColVector&): invalid vector dimension");

  return this->moveBaseWorldFrame(vpHomogeneousMatrix(v[0], v[1], v[2], v[3], v[4], v[5]));
}

bool usNeedleInsertionModelInterface::moveBaseWorldFrame(const vpHomogeneousMatrix &Hmotion)
{
  // Move base
  // Translation is expressed in world frame,
  // Rotation is expressed in world frame but is done around the base position

  vpHomogeneousMatrix Hrotation0(this->getWorldMbase());

  Hrotation0[0][3] = 0;
  Hrotation0[1][3] = 0;
  Hrotation0[2][3] = 0;

  vpHomogeneousMatrix HmotionBaseFrame = Hrotation0.inverse() * Hmotion * Hrotation0;

  return this->moveBase(HmotionBaseFrame);
}
