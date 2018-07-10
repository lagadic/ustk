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
  \example testUsNeedleInsertionModelKinematic.cpp

  USTK usNeedleInsertionModelKinematic test

  This example tests the model of the insertion of a needle via the class usNeedleInsertionModelKinematic.
*/

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelKinematic.h>

#include <visp3/ustk_needle_modeling/usNeedleModelingDisplayTools.h>

#include <visp3/gui/vpDisplayX.h>


int main()
{  
    usNeedleInsertionModelKinematic n;
    n.accessNeedle().setBasePose(vpPoseVector(0,0,0.1, M_PI/sqrt(2),M_PI/sqrt(2),0));
    n.accessNeedle().setTipPose(vpPoseVector(0,0,0, M_PI/sqrt(2),M_PI/sqrt(2),0));
    n.setNaturalCurvature(1/0.05);
    
    usNeedleInsertionModelKinematic n1;
    n1.accessNeedle().setBasePose(vpPoseVector(0.01,0,0.1, M_PI/sqrt(2),M_PI/sqrt(2),0));
    n1.accessNeedle().setTipPose(vpPoseVector(0.01,0,0, M_PI/sqrt(2),M_PI/sqrt(2),0));
    n1.setNaturalCurvature(1/0.2);
    
    usOrientedPlane3D surface(n.accessNeedle().getTipPose());
    
    vpImage<unsigned char> I1(700, 500, 255);
    vpDisplayX d1(I1);
    
    for(int i=0 ; i<1000 ; i++)
    {        
        n.moveBase(0,0,0.0001,0,0,0.01);
        n1.moveBase(0,0,0.0001,0,0,0.01);
        
        vpDisplay::display(I1);
        
        usNeedleModelingDisplayTools::display(n, I1, vpHomogeneousMatrix(0.08 ,0.1, 0.2, M_PI/2,0,0), 3000,3000);
        usNeedleModelingDisplayTools::display(n1, I1, vpHomogeneousMatrix(0.08 ,0.1, 0.2, M_PI/2,0,0), 3000,3000);
        usGeometryDisplayTools::display(surface, I1, vpHomogeneousMatrix(0.08 ,0.1, 0.2, M_PI/2,0,0), 3000,3000);
        
        vpDisplay::flush(I1);
    }

    return 0;
}
