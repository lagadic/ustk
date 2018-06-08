/****************************************************************************
 *
 * This file is part of the UsNeedleDetection software.
 * Copyright (C) 2013 - 2016 by Inria. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

/*!
  \example testUsGeometryDisplayTools.cpp

  USTK usGeometryDisplayTools test

  This example tests all functions declared in the usGeometryDisplayTools namespace.
*/

#include <visp3/ustk_needle_detection/usGeometryDisplayTools.h>

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

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>


int main()
{
    std::cout << "Start testUsGeometryDisplayTools" << std::endl;
    
    vpImage<unsigned char> I1(700, 500, 255);
    vpImage<vpRGBa> I2(700, 500, vpRGBa(255,255,255,255));
            
#if defined(VISP_HAVE_X11)
    vpDisplayX d1(I1, 0,0, "Display unsigned char");
    vpDisplayX d2(I2, d1.getWindowXPosition()+d1.getWidth(), d1.getWindowYPosition(), "Display vpRGBa");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d1(I1, "Display unsigned char");
    vpDisplayOpenCV d2(I2, d1.getWindowXPosition()+d1.getWidth(), d1.getWindowYPosition(), "Display vpRGBa");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d1(I1, "Display unsigned char");
    vpDisplayGTK d2(I2, d1.getWindowXPosition()+d1.getWidth(), d1.getWindowYPosition(), "Display vpRGBa");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d1(I1, "Display unsigned char");
    vpDisplayGDI d2(I2, d1.getWindowXPosition()+d1.getWidth(), d1.getWindowYPosition(), "Display vpRGBa");
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3d d1(I1, "Display unsigned char");
    vpDisplayD3d d2(I2, d1.getWindowXPosition()+d1.getWidth(), d1.getWindowYPosition(), "Display vpRGBa");
#else
    std::cout << "No image viewer is available..." << std::endl;
    return 0;
#endif
        
    usOrientedPlane3D plane(vpPoseVector(0,0,0.05, 0,M_PI/4,0));
    
    std::vector<vpColVector> points2D(4, vpColVector(2,0));
    points2D.at(0)[0] = 0.02;points2D.at(0)[1] = 0.02;
    points2D.at(1)[0] = 0.02;points2D.at(1)[1] = 0.03;
    points2D.at(2)[0] = 0.03;points2D.at(2)[1] = 0.03;
    points2D.at(3)[0] = 0.02;points2D.at(3)[1] = 0.05;
    usPolynomialCurve2D curve2D;
    curve2D.defineFromPointsAuto(points2D, points2D.back()-points2D.front(),3);
    
    std::vector<vpColVector> points3D(4, vpColVector(3,0));
    points3D.at(1)[0] = 0.02;
    points3D.at(2)[1] = 0.02;points3D.at(2)[2] = 0.03;
    points3D.at(3)[2] = 0.05;
    usPolynomialCurve3D curve3D;
    curve3D.defineFromPointsAuto(points3D, points3D.back()-points3D.front(),3);
        
    usBSpline3D spline;
    std::vector<double> lengths(3);
    lengths.at(0) = 0.02;lengths.at(1) = 0.04;lengths.at(2) = 0.04;
    spline.defineFromPoints(points3D, lengths, 3);
    
    vpHomogeneousMatrix imageMworld(0.1, 0.1, 0.1, M_PI/2, 0, 0);

    vpDisplay::display(I1);
    vpDisplay::display(I2);
    
    usGeometryDisplayTools::display(plane, I1, imageMworld, 3000, 3000, vpColor::green);
    std::cout << "done: usGeometryDisplayTools::display(const usOrientedPlane3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    usGeometryDisplayTools::display(plane, I2, imageMworld, 3000, 3000, vpColor::green);
    std::cout << "done: usGeometryDisplayTools::display(const usOrientedPlane3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    
    usGeometryDisplayTools::display(curve2D, I1, 3000, 3000, vpColor::red, 10);
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve2D&, vpImage<unsigned char>&, double, double, const vpColor&);" << std::endl;
    usGeometryDisplayTools::display(curve2D, I2, 3000, 3000, vpColor::red, 10);
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve2D&, vpImage<vpRGBa>&, double, double, const vpColor&);" << std::endl;
    
    usGeometryDisplayTools::display(curve3D, I1, imageMworld, 3000, 3000, vpColor::blue, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    usGeometryDisplayTools::display(curve3D, I2, imageMworld, 3000, 3000, vpColor::blue, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    
    usGeometryDisplayTools::displayLine(spline, I1, imageMworld, 3000, 3000, vpColor::orange, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayLine(const usBSpline3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    usGeometryDisplayTools::displayLine(spline, I2, imageMworld, 3000, 3000, vpColor::orange, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayLine(const usBSpline3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    
    usGeometryDisplayTools::displayExtremities(spline, I1, imageMworld, 3000, 3000, vpColor::orange, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayExtremities(const usBSpline3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, double);" << std::endl;
    usGeometryDisplayTools::displayExtremities(spline, I2, imageMworld, 3000, 3000, vpColor::orange, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayExtremities(const usBSpline3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, double);" << std::endl;
    
    usGeometryDisplayTools::displayCurvatureFromShape(spline, I1, imageMworld, 3000, 3000, vpColor::orange);
    std::cout << "done: usGeometryDisplayTools::displayCurvatureFromShape(const usBSpline3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    usGeometryDisplayTools::displayCurvatureFromShape(spline, I2, imageMworld, 3000, 3000, vpColor::orange);
    std::cout << "done: usGeometryDisplayTools::displayCurvatureFromShape(const usBSpline3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    
    vpDisplay::flush(I1);
    vpDisplay::flush(I2);
    
    vpTime::wait(2000);
    
    return 0;
}
