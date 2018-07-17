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

#ifndef __usNeedleInsertionModelInterface_h
#define __usNeedleInsertionModelInterface_h

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>

class VISP_EXPORT usNeedleInsertionModelInterface
{
public:
  //! The following methods should be redefined in the derived classes

  virtual bool setBasePose(
      const vpPoseVector &pose) = 0; // must set the base pose and automatically update the whole state of the model
  virtual vpPoseVector getBasePose() const = 0;

  //! These function simply overload the previous ones
  bool setBasePose(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  bool setBasePose(const vpHomogeneousMatrix &Hpose);

  vpHomogeneousMatrix getWorldMbase() const;
  vpColVector getBasePosition() const;

  bool moveBase(const vpColVector &v, double time);
  bool moveBase(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  bool moveBase(const vpPoseVector &pose);
  bool moveBase(const vpColVector &v);
  bool moveBase(const vpHomogeneousMatrix &Hmotion);

  bool moveBaseWorldFrame(const vpColVector &command, double time);
  bool moveBaseWorldFrame(double tx, double ty, double tz, double thetax, double thetay, double thetaz);
  bool moveBaseWorldFrame(const vpPoseVector &pose);
  bool moveBaseWorldFrame(const vpColVector &v);
  bool moveBaseWorldFrame(const vpHomogeneousMatrix &Hmotion);
};

#endif // __usNeedleInsertionModelInterface_h
