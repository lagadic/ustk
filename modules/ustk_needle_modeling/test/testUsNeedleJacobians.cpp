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

/*!
  \example testUsNeedleJacobians.cpp

  USTK usNeedleJacobians test

  This example tests the Jacobian computation associated to the needle insertion models.
*/

#include <visp3/core/vpConfig.h>

#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelRayleighRitzSpline.h>
#include <visp3/ustk_needle_modeling/usNeedleInsertionModelVirtualSprings.h>

#include <visp3/ustk_needle_modeling/usNeedleJacobians.h>

int main()
{
    std::cout << "Start test testUsNeedleJacobians" << std::endl;
    
    //! Generate some inserted needle models

    usNeedleInsertionModelRayleighRitzSpline needleRR;
    needleRR.loadPreset(usNeedleInsertionModelRayleighRitzSpline::ModelPreset::BiopsyNeedle);
    needleRR.setPathUpdateType(usNeedleInsertionModelRayleighRitzSpline::PathUpdateType::WithTipPosition);
    needleRR.setBasePose(vpPoseVector(0,0,0,0,0,0));
    needleRR.setSurfaceAtTip();
    usNeedleInsertionModelVirtualSprings needleVS;
    needleVS.loadPreset(usNeedleInsertionModelVirtualSprings::ModelPreset::BiopsyNeedle);
    needleVS.setBasePose(vpPoseVector(0,0,0,0,0,0));
    needleVS.setSurfaceAtTip();
    
    for(int i=0 ; i<20 ; i++)
    {
      needleRR.moveBase(0.0005,0,0.001,0.0001,0,0);
      needleVS.moveBase(0.0005,0,0.001,0.0001,0,0);
    }
    
    vpMatrix JRR;
    vpMatrix JVS;
    
    //! Base to tip (manipulation Jacobian) (6x6)
       
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToWorldTipVelocity(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToWorldTipVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToWorldTipVelocity(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToWorldTipVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToTipVelocity(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToTipVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToTipVelocity(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTipVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToWorldTipVelocity(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToWorldTipVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToWorldTipVelocity(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToWorldTipVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToTipVelocity(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToTipVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToTipVelocity(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTipVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;

    //! Base to bending energy (1x6)
    
    if(!usNeedleJacobians::getJacobianBaseVelocityToBendingEnergy(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToBendingEnergy<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToBendingEnergy(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToBendingEnergy<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    
    if(!usNeedleJacobians::getJacobianBaseVelocityToBendingEnergy(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToBendingEnergy<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToBendingEnergy(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToBendingEnergy<usNeedleInsertionModelVirtualSprings>" << std::endl;
    
    //! Base to tissue deformation energy (1x6)
    
    if(!usNeedleJacobians::getJacobianBaseVelocityToTissueEnergy(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTissueEnergy<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToTissueEnergy(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToTissueEnergy<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToNeedleInsertionPointVelocity(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToNeedleInsertionPointVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToSurfaceTissueStretch(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToSurfaceTissueStretch<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToMaxTissueStretch(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToMaxTissueStretch<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToMeanTissueStretch(needleRR, JRR)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToMeanTissueStretch<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    
    if(!usNeedleJacobians::getJacobianBaseVelocityToTissueEnergy(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTissueEnergy<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToTissueEnergy(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToTissueEnergy<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToNeedleInsertionPointVelocity(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToNeedleInsertionPointVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToSurfaceTissueStretch(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToSurfaceTissueStretch<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToMaxTissueStretch(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToMaxTissueStretch<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToMeanTissueStretch(needleVS, JVS)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToMeanTissueStretch<usNeedleInsertionModelVirtualSprings>" << std::endl;
    
    //! Base to needle point (3x6)
    
    if(!usNeedleJacobians::getJacobianBaseVelocityToPointVelocity(needleRR, JRR, 0.5*needleRR.accessNeedle().getFullLength())) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToPointVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToPointVelocity(needleRR, JRR, 0.5*needleRR.accessNeedle().getFullLength())) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToPointVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    
    if(!usNeedleJacobians::getJacobianBaseVelocityToPointVelocity(needleVS, JVS, 0.5*needleRR.accessNeedle().getFullLength())) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToPointVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianWorldBaseVelocityToPointVelocity(needleVS, JVS, 0.5*needleRR.accessNeedle().getFullLength())) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianWorldBaseVelocityToPointVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
    
    //! Base to features (1x6)
    
    vpColVector target(needleRR.accessNeedle().getTipPosition());
    target[0] += 0.01;
    target[2] += 0.02;
    if(!usNeedleJacobians::getJacobianBaseVelocityToTransversalTargetDistance(needleRR, JRR, target)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTransversalTargetDistance<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToTargetAngle(needleRR, JRR, target)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTargetAngle<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    
    if(!usNeedleJacobians::getJacobianBaseVelocityToTransversalTargetDistance(needleVS, JVS, target)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTransversalTargetDistance<usNeedleInsertionModelVirtualSprings>" << std::endl;
    if(!usNeedleJacobians::getJacobianBaseVelocityToTargetAngle(needleVS, JVS, target)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianBaseVelocityToTargetAngle<usNeedleInsertionModelVirtualSprings>" << std::endl;
    
    //! Springs to needle point (3x6)
    
    if(!usNeedleJacobians::getJacobianSpringVelocityToPointVelocity(needleVS, JVS, needleVS.getNbSprings()/2, 0.5 * needleVS.accessNeedle().getFullLength())) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianSpringVelocityToPointVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
    
    //! Springs to distance from points (nbPointsx6)
    
    vpColVector point(needleVS.accessNeedle().getPoint(0.5*needleVS.accessNeedle().getFullLength()));
    point[0] += 0.005;
    std::vector<vpColVector> points;
    for(int i=0 ; i<10 ; i++)
    {
      point[2] += 0.002;
      points.push_back(point);
    }
    if(!usNeedleJacobians::getJacobianSpringVelocityToPointsDistance(needleVS, JVS, needleVS.getNbSprings()/2, points)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianSpringVelocityToPointsDistance<usNeedleInsertionModelVirtualSprings>" << std::endl;
    
    //! Rest path points to model point (3*nbPointsx3)
    
    std::vector<double> l;
    for(int i=0 ; i<10 ; i++) l.push_back(i * needleRR.accessNeedle().getFullLength() / 10.);
    if(!usNeedleJacobians::getJacobianRestPathPointVelocityToPointsVelocity(needleRR, JRR, needleRR.accessTissue().accessPath().getNbSegments() / 2, l)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianRestPathPointVelocityToPointsVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    
    //! Rest path points distance from points (nbPointsx3)
    
    if(!usNeedleJacobians::getJacobianRestPathPointVelocityToPointsDistance(needleRR, JRR, needleRR.accessTissue().accessPath().getNbSegments() / 2, points)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianRestPathPointVelocityToPointsDistance<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    
    //! Stiffness per unit length to distance from points (nbPointsx1)
    
    if(!usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsDistance(needleRR, JRR, points)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsDistance<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsDistance(needleVS, JVS, points)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsDistance<usNeedleInsertionModelVirtualSprings>" << std::endl;
    
    //! Stiffness per unit length to needle point ( (3*nbPoints)x1 matrix)
    
    if(!usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsVelocity(needleRR, JRR, l)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsVelocity<usNeedleInsertionModelRayleighRitzSpline>" << std::endl;
    if(!usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsVelocity(needleVS, JVS, l)) return 1;
    std::cout << "done: usNeedleJacobians::getJacobianStiffnessPerUnitLengthToPointsVelocity<usNeedleInsertionModelVirtualSprings>" << std::endl;
        
    return 0;
}
