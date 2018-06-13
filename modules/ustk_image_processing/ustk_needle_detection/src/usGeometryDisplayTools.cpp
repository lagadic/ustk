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

#include <visp3/ustk_needle_detection/usGeometryDisplayTools.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>


namespace usGeometryDisplayTools
{

template<class ImageDataType>
void displayFrame(const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMframe, double Xscale, double Yscale)
{
    double origin_x = Xscale * imageMframe[0][3];
    double origin_y = Yscale * imageMframe[1][3];

    double zmax = -2;
    double zmin = 2;
    int zmax_index = 0;
    int zmin_index = 0;

    for(int j=0 ; j<3 ; j++)
    {
        double z = imageMframe[2][j];
        if(z>=zmax)
        {
            zmax = z;
            zmax_index = j;
        }
        if(z<=zmin)
        {
            zmin = z;
            zmin_index = j;
        }
    }
    double order[3];
    order[0] = zmax_index;
    order[1] = 3-zmin_index-zmax_index;
    order[2] = zmin_index;

    // Display frame
    vpColor color[3] = {vpColor::red, vpColor::green, vpColor::blue};
    for(int i=0; i<3 ; i++)
    {
        int j = order[i];
        vpDisplay::displayArrow(I, origin_y, origin_x, origin_y+20*imageMframe[1][j], origin_x+20*imageMframe[0][j], color[j], 5, 5, 1);
    }
}
template void displayFrame<unsigned char>(const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayFrame<vpRGBa>(const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usOrientedPlane3D

template <class ImageDataType>
void display(const usOrientedPlane3D &plane, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, const vpColor &color)
{
    vpMatrix R(imageMworld.getRotationMatrix());
    vpColVector T(imageMworld.getTranslationVector());
    
    vpColVector imagePoint(R * plane.getPosition() + T);
    vpColVector imageDirection = R * plane.getDirection();
    
    double x = Xscale * imagePoint[0];
    double y = Yscale * imagePoint[1];

    double dx = 0.1 * Xscale * imageDirection[0];
    double dy = 0.1 * Yscale * imageDirection[1];

    vpDisplay::displayCross(I, y, x, 7, color);
    vpDisplay::displayLine(I, y-dx, x+dy, y+dx, x-dy , color);
}
template void display(const usOrientedPlane3D&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&);
template void display(const usOrientedPlane3D&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&);

//! Display usPolynomialCurve2D

template<class ImageDataType>
void display(const usPolynomialCurve2D &curve, const vpImage<ImageDataType> &I, double Xscale, double Yscale, const vpColor &color, int nbRenderingLines)
{
    double step = (curve.getEndParameter()-curve.getStartParameter())/nbRenderingLines;
    
    std::vector<double> params(nbRenderingLines+1);
    params.front() = curve.getStartParameter();
    for(int i=1 ; i<nbRenderingLines+1 ; i++) params.at(i) = params.at(i-1)+step;
    
    vpMatrix imagePoints = curve.getPoints(params);

    double x0 = Xscale*imagePoints[0][0];
    double y0 = Yscale*imagePoints[1][0];
    double x1 = x0;
    double y1 = y0;

    for(int i=0 ; i<nbRenderingLines ; i++)
    {
        x0 = x1;
        y0 = y1;

        x1 = Xscale * imagePoints[0][i+1];
        y1 = Yscale * imagePoints[1][i+1];

        vpDisplay::displayLine(I, y0,x0, y1,x1, color);
    }
}
template void display<unsigned char>(const usPolynomialCurve2D&, const vpImage<unsigned char>&, double, double, const vpColor&, int);
template void display<vpRGBa>(const usPolynomialCurve2D&, const vpImage<vpRGBa>&, double, double, const vpColor&, int);

//! Display usPolynomialCurve3D

template<class ImageDataType>
void display(const usPolynomialCurve3D &curve, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, const vpColor &color, int nbRenderingLines, double visibilityDistance)
{
    unsigned int nbPoints = nbRenderingLines+1;
    double step = (curve.getEndParameter()-curve.getStartParameter())/nbRenderingLines;
    
    std::vector<double> params(nbPoints);
    params.front() = curve.getStartParameter();
    for(unsigned int i=1 ; i<nbPoints ; i++) params.at(i) = params.at(i-1)+step;
    
    vpMatrix points = curve.getPoints(params);
    
    vpMatrix R(imageMworld.getRotationMatrix());
    vpColVector T(imageMworld.getTranslationVector());
    vpMatrix imagePoints((vpMatrix)R * points + vpMatrix::kron(T, vpMatrix(1,nbPoints,1)));

    double x0 = Xscale*imagePoints[0][0];
    double y0 = Yscale*imagePoints[1][0];
    double z0 = imagePoints[2][0];
    double x1 = x0;
    double y1 = y0;
    double z1 = z0;

    for(int i=0 ; i<nbRenderingLines ; i++)
    {
        x0 = x1;
        y0 = y1;
        z0 = z1;

        x1 = Xscale * imagePoints[0][i+1];
        y1 = Yscale * imagePoints[1][i+1];
        z1 = imagePoints[2][i+1];

        if( (vpMath::sign(z0)!=vpMath::sign(z1)) || ((fabs(z0)<visibilityDistance)&&(fabs(z1)<visibilityDistance)) ) vpDisplay::displayLine(I, y0,x0, y1,x1, color);
    }
}

template void display<unsigned char>(const usPolynomialCurve3D&, const vpImage<unsigned char>&, const vpHomogeneousMatrix &, double, double, const vpColor &, int, double);
template void display<vpRGBa>(const usPolynomialCurve3D&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix &, double, double, const vpColor &, int, double);

template<class ImageDataType>
void displayCurvatureFromShape(const usPolynomialCurve3D &curve, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, const vpColor &color)
{
    vpColVector C(4);
    vpColVector D(4);
    double k = curve.getCurvatureFromShape(0, -1, C, D);
    if(k == 0) return;

    double r = 1/k;

    C = imageMworld * C;
    C.resize(3,false);
    D = imageMworld * D;
    D.resize(3,false);

    vpColVector Dxy(3,0);
    Dxy[2] = 1;
    vpColVector Axy = vpColVector::cross(D,Dxy);
    vpColVector Bxy = vpColVector::cross(D,Axy).normalize();

    double thetaxy = atan2(Axy[1], Axy[0]);
    double axy = r;
    double bxy = r * vpColVector::cross(Bxy,Dxy).euclideanNorm();
    if(Bxy.euclideanNorm() < 0.5) bxy = axy; // if normalization failed the circle is almost in the plane

    //Scaling
    vpImagePoint centerXY(Yscale*C[1], Xscale*C[0]);
    double A = sqrt( pow(Xscale * cos(thetaxy),2) + pow(Yscale * sin(thetaxy),2) );
    axy *= A;
    bxy *= A;
    thetaxy = atan2(axy*sin(thetaxy), bxy*cos(thetaxy));
    vpDisplay::displayEllipse(I, centerXY, axy, bxy, thetaxy, false, color);
}
template void displayCurvatureFromShape<unsigned char>(const usPolynomialCurve3D&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&);
template void displayCurvatureFromShape<vpRGBa>(const usPolynomialCurve3D&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&);

//! Display usBSpline3D

template<class ImageDataType>
void displayLine(const usBSpline3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, const vpColor &color, int nbRenderingLinesPerSegment, double visibilityDistance)
{
    for(int i=0 ; i<spline.getNbSegments() ; i++) usGeometryDisplayTools::display(spline.accessSegment(i), I, imageMworld, Xscale, Yscale, color, nbRenderingLinesPerSegment, visibilityDistance);
}
template void displayLine<unsigned char>(const usBSpline3D&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);
template void displayLine<vpRGBa>(const usBSpline3D&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);

template<class ImageDataType>
void displayExtremities(const usBSpline3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, const vpColor &color, double visibilityDistance)
{
    unsigned int nbPoints = spline.getNbSegments()+1;
    if(nbPoints < 2) return;
    
    vpMatrix points(3, nbPoints);
    
    for(int i=0 ; i<spline.getNbSegments() ; i++) points.insert(spline.accessSegment(i).getStartPoint(), 0,i);
    points.insert(spline.accessLastSegment().getEndPoint(), 0,spline.getNbSegments());
        
    vpMatrix R(imageMworld.getRotationMatrix());
    vpColVector T(imageMworld.getTranslationVector());
    vpMatrix imagePoints((vpMatrix)R * points + vpMatrix::kron(T, vpMatrix(1,nbPoints,1)));

    for(unsigned int i=0 ; i<nbPoints ; i++)
    {
        double x = Xscale * imagePoints[0][i];
        double y = Yscale * imagePoints[1][i];
        double z = imagePoints[2][i];
        if(fabs(z)<visibilityDistance) vpDisplay::displayCross(I, y,x, 7, color);
    }
}
template void displayExtremities<unsigned char>(const usBSpline3D&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, double);
template void displayExtremities<vpRGBa>(const usBSpline3D&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, double);

template<class ImageDataType>
void display(const usBSpline3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, const vpColor &color, int nbRenderingLines, double visibilityDistance)
{
    displayLine(spline, I, imageMworld, Xscale, Yscale, color, nbRenderingLines, visibilityDistance);
    displayExtremities(spline, I, imageMworld, Xscale, Yscale, color, visibilityDistance);      
}
template void display<unsigned char>(const usBSpline3D&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);
template void display<vpRGBa>(const usBSpline3D&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);

template<class ImageDataType>
void displayCurvatureFromShape(const usBSpline3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, const vpColor &color)
{
    vpColVector C(4);
    vpColVector D(4);
    double k = spline.getCurvatureFromShape(0, -1, C, D);
    if(k == 0) return;

    double r = 1/k;

    C = imageMworld * C;
    C.resize(3,false);
    D = imageMworld * D;
    D.resize(3,false);

    vpColVector Dxy(3,0);
    Dxy[2] = 1;
    vpColVector Axy = vpColVector::cross(D,Dxy);
    vpColVector Bxy = vpColVector::cross(D,Axy).normalize();

    double thetaxy = atan2(Axy[1], Axy[0]);
    double axy = r;
    double bxy = r * vpColVector::cross(Bxy,Dxy).euclideanNorm();
    if(Bxy.euclideanNorm() < 0.5) bxy = axy; // if normalization failed the circle is almost in the plane

    //Scaling
    vpImagePoint centerXY(Yscale*C[1], Xscale*C[0]);
    double A = sqrt( pow(Xscale * cos(thetaxy),2) + pow(Yscale * sin(thetaxy),2) );
    axy *= A;
    bxy *= A;
    thetaxy = atan2(axy*sin(thetaxy), bxy*cos(thetaxy));
    vpDisplay::displayEllipse(I, centerXY, axy, bxy, thetaxy, false, color);
}
template void displayCurvatureFromShape<unsigned char>(const usBSpline3D&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&);
template void displayCurvatureFromShape<vpRGBa>(const usBSpline3D&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&);

} // namespace usGeometryDisplayTools
