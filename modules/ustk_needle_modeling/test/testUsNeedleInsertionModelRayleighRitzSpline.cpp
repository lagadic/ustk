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
  \example testUsNeedleInsertionModelRayleighRitzSpline.cpp

  USTK usNeedleInsertionModelRayleighRitzSpline test

  This example tests the model of the insertion of a needle via the class usNeedleInsertionModelRayleighRitzSpline.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9)

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelRayleighRitzSpline.h>
#include <visp3/ustk_needle_modeling/usNeedleModelingDisplayTools.h>

#if defined(VISP_HAVE_X11)
#include <visp3/gui/vpDisplayX.h>
#elif defined(VISP_HAVE_OPENCV)
#include <visp3/gui/vpDisplayOpenCV.h>
#elif defined(VISP_HAVE_GTK)
#include <visp3/gui/vpDisplayGTK.h>
#elif defined(VISP_HAVE_GDI)
#include <visp3/gui/vpDisplayGDI.h>
#elif defined(VISP_HAVE_D3D9)
#include <visp3/gui/vpDisplayD3D.h>
#endif

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPoseVector.h>


int main()
{
    usNeedleInsertionModelRayleighRitzSpline n;

    n.loadPreset(usNeedleInsertionModelRayleighRitzSpline::ModelPreset::BiopsyNeedle);
    n.accessNeedle().setOuterDiameter(0.001);
    n.accessNeedle().setInsideDiameter(0.0007);
    n.accessNeedle().setNeedleYoungModulus(75e9);
    n.accessNeedle().setFullLength(0.1);
    
    n.accessNeedle().accessSegment(0).setOrder(3);
    n.setStiffnessPerUnitLength(0, 10000);
    
    n.setPathUpdateType(usNeedleInsertionModelRayleighRitzSpline::PathUpdateType::WithTipPosition);
    n.setPathUpdateLengthThreshold(0.001);
        
    n.setBasePose(vpPoseVector(0,0,0.1, M_PI,0,0));

    n.accessTissue().accessSurface().setPosition(n.accessNeedle().getTipPosition()+0.01*n.accessNeedle().getTipDirection());
    n.accessTissue().accessSurface().setDirection(n.accessNeedle().getTipDirection());

    n.setNeedleTipType(usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::ActuatedTip);
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setDiameter(2*n.accessNeedle().getOuterDiameter());
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setLength(0.005);
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setTipAngleDeg(5);
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setSteeringAngleDeg(90);
    
    n.setSolvingMethod(usNeedleInsertionModelRayleighRitzSpline::SolvingMethod::FixedBeamLength);
    
    n.updateState();

    usNeedleInsertionModelRayleighRitzSpline n1(n);
    n1.moveBaseWorldFrame(0.01,0,0,0,0,0);
    
    vpImage<unsigned char> I1(700, 500, 255);
    
#if defined(VISP_HAVE_X11)
    vpDisplayX *d1;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV *d1;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK *d1;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI *d1;
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D *d1;
#endif
    
    try
    {
#if defined(VISP_HAVE_X11)
    d1 = new vpDisplayX(I1);
#elif defined(VISP_HAVE_OPENCV)
    d1 = new vpDisplayOpenCV(I1);
#elif defined(VISP_HAVE_GTK)
    d1 = new vpDisplayGTK(I1);
#elif defined(VISP_HAVE_GDI)
    d1 = new vpDisplayGDI(I1);
#elif defined(VISP_HAVE_D3D9)
    d1 = new vpDisplayD3D(I1);
#endif
    }
    catch(std::exception &e)
    {
        std::cout << "testUsNeedleInsertionModelRayleighRitzSpline: could not initialize display:\n" << e.what() << std::endl;
        return 0;
    }
    
    for(int i=0 ; i<3000 ; i++)
    {        
        if(i > 500 && i < 1000)
        {
            n1.accessNeedle().setFullLength(n1.accessNeedle().getFullLength()+0.00002);
            n1.updateState();
        }
        
        if(i > 500)
        {
            dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setTipAngleDeg(0.1*(i%400));            
            dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setSteeringAngleDeg(i%360);
            n.updateState();
        }

        n.moveBase(0,0,0.0001,0,0,0.01);
        n1.moveBase(0,0,0.0001,0,0,0.01);
        
        vpDisplay::display(I1);
        
        usNeedleModelingDisplayTools::display(n, I1, vpHomogeneousMatrix(0.08 ,0.1, 0.2, M_PI/2,0,0), 3000,3000);
        usNeedleModelingDisplayTools::display(n1, I1, vpHomogeneousMatrix(0.08 ,0.1, 0.2, M_PI/2,0,0), 3000,3000);
        
        vpDisplay::flush(I1);
    }

    delete d1;

    return 0;
}

#else

#include <iostream>

int main()
{
    std::cout << "No display to start testUsNeedleInsertionModelRayleighRitzSpline" << std::endl;
    
    return 0;
}

#endif

