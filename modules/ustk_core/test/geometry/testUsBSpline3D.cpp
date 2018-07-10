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
  \example testUsBSpline3D.cpp

  USTK usBSpline3D test

  This example tests all functions declared in the usBSpline3D class.
*/

#include <visp3/ustk_core/usBSpline3D.h>

#include <vector>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>


int main()
{
    std::cout << "Start testUsBSpline3D" << std::endl;
    usBSpline3D spline;
    std::cout << "done: usBSpline3D default contructor" << std::endl;
    usBSpline3D spline2(spline);
    std::cout << "done: usBSpline3D copy contructor" << std::endl;
    spline = spline2;
    std::cout << "done: usBSpline3D assignment operator" << std::endl;
    usBSpline3D *spline3 = spline.clone();
    std::cout << "done: usBSpline3D polymorphic copy" << std::endl;
    
    std::vector<vpColVector> points(4, vpColVector(3,0));
    points.at(1)[0] = 1;points.at(2)[1] = 2;points.at(3)[2] = 1;
    std::vector<double> params(4, 0);
    for(int i=0 ; i<4 ; i++) params[i] = i;
    usPolynomialCurve3D curve1;
    curve1.defineFromPoints(points, params, 3);
    usPolynomialCurve3D curve2(curve1);
    curve2.move(1,0.5,0, 1,0,0);
    usPolynomialCurve3D curve3(curve2);
    curve3.move(-0.5,0,-0.5, 0,1,0.5);
    
    spline.addSegment(curve1);
    std::cout << "done: usBSpline3D::addSegment" << std::endl;
    spline.addSegment(curve3);
    spline.insertSegment(1, curve1);
    std::cout << "done: usBSpline3D::insertSegment" << std::endl;
    
    spline.setSegment(1, curve2);
    std::cout << "done: usBSpline3D::setSegment" << std::endl;
    
    spline.removeSegment(1);
    std::cout << "done: usBSpline3D::removeSegment" << std::endl;
    spline.removeLastSegment();
    std::cout << "done: usBSpline3D::removeLastSegment" << std::endl;
    
    spline.addSegment(curve2);
    spline.addSegment(curve3);
    spline.removeSegments(1,2);
    std::cout << "done: usBSpline3D::removeSegments" << std::endl;
    
    spline.getNbSegments();
    std::cout << "done: usBSpline3D::getNbSegments" << std::endl;
    spline.getParametricLength();
    std::cout << "done: usBSpline3D::getParametricLength" << std::endl;
    spline.getLength(100);
    std::cout << "done: usBSpline3D::getLength" << std::endl;

    spline.clear(); 
    std::cout << "done: usBSpline3D::clear" << std::endl;

    std::vector<double> lengths(3, 1);
    spline.defineFromPoints(points, lengths, 3);
    std::cout << "done: usBSpline3D::defineFromPoints" << std::endl;

    curve2 = spline.accessSegment(1);
    std::cout << "done: usBSpline3D::accessSegment const" << std::endl;
    curve3 = spline.accessLastSegment();
    std::cout << "done: usBSpline3D::accessLastSegment const" << std::endl;
    spline.accessSegment(1) = curve2;
    std::cout << "done: usBSpline3D::addSegment" << std::endl;
    spline.accessLastSegment() = curve3;
    std::cout << "done: usBSpline3D::addSegment" << std::endl;
    *spline3 = spline.getSubSpline(0.5, 1.5);
    std::cout << "done: usBSpline3D::getSubSpline" << std::endl;

    spline.move(vpHomogeneousMatrix(0.5, -0.25, 0.1, 1, -1, 0.2));
    std::cout << "done: usBSpline3D::move(const vpHomogeneousMatrix&)" << std::endl;
    spline.move(0.5, -0.25, 0.1, 1, -1, 0.2);
    std::cout << "done: usBSpline3D::move(double,double,double,double,double,double)" << std::endl;


    spline.getPoint(1.2);
    std::cout << "done: usBSpline3D::getPoint" << std::endl;
    spline.getTangent(1.2);
    std::cout << "done: usBSpline3D::getTangent" << std::endl;
    spline.getDistanceFromPoint(points.at(1), 0, -1, 1e-6);
    std::cout << "done: usBSpline3D::getDistanceFromPoint" << std::endl;
    int index;
    double param;
    spline.getParametersFromLength(1.5, index, param);
    std::cout << "done: usBSpline3D::getParametersFromLength" << std::endl;

    vpColVector center3D;
    vpColVector direction3D;
    spline.getCurvatureFromShape(0,1, center3D, direction3D);
    std::cout << "done: usBSpline3D::getCurvatureFromShape" << std::endl;

    delete spline3;
    std::cout << "done: usBSpline3D::~usBSpline3D" << std::endl;
            
    return 0;
}
