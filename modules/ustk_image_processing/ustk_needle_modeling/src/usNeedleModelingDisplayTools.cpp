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


#include <visp3/ustk_needle_modeling/usNeedleModelingDisplayTools.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/ustk_needle_detection/usGeometryTools.h>


namespace usNeedleModelingDisplayTools
{

//! Display usNeedleModelBaseTip

template<class ImageDataType>
void displayBase(const usNeedleModelBaseTip &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::displayFrame(I, imageMworld * needleModel.getWorldMbase(), Xscale, Yscale);
}
template void displayBase<unsigned char>(const usNeedleModelBaseTip&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayBase<vpRGBa>(const usNeedleModelBaseTip&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void displayTip(const usNeedleModelBaseTip &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::displayFrame(I, imageMworld * needleModel.getWorldMtip(), Xscale, Yscale);
}
template void displayTip<unsigned char>(const usNeedleModelBaseTip&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayTip<vpRGBa>(const usNeedleModelBaseTip&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void display(const usNeedleModelBaseTip &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    displayBase(needleModel, I, imageMworld, Xscale, Yscale);
    displayTip(needleModel, I, imageMworld, Xscale, Yscale);
}
template void display(const usNeedleModelBaseTip &needleModel, const vpImage<unsigned char> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale);
template void display(const usNeedleModelBaseTip &needleModel, const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale);

//! Display usNeedleModelPolynomial

template<class ImageDataType>
void displayNeedle(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, bool displayFullBody)
{
    usGeometryDisplayTools::display((const usPolynomialCurve3D&)needleModel, I, imageMworld, Xscale, Yscale, vpColor::red, 10);
    if(displayFullBody)
    {
        double r = 0.5 * needleModel.getOuterDiameter();
        double l = needleModel.getParametricLength();
        double step = l / 10;
        vpColVector z(imageMworld.inverse().getCol(2));

        std::vector<double> params(11);
        std::vector<vpColVector> p1(11);
        std::vector<vpColVector> p2(11);
        for(int i=0 ; i<11 ; i++)
        {
            params.at(i) = i * step;
            vpColVector p(needleModel.getPoint(params.at(i)));
            vpColVector x(vpColVector::crossProd(needleModel.getTangent(params.at(i)), z).normalize());
            p1.at(i) = p + r * x;
            p2.at(i) = p - r * x;
        }
        vpColVector p(needleModel.getEndPoint());
        vpColVector x(vpColVector::crossProd(needleModel.getEndTangent(), z).normalize());
        p1.back() = p + r * x;
        p2.back() = p - r * x;
        usPolynomialCurve3D s;
        s.defineFromPoints(p1, params, needleModel.getOrder());
        usGeometryDisplayTools::display(s, I, imageMworld, Xscale, Yscale);
        s.defineFromPoints(p2, params, needleModel.getOrder());
        usGeometryDisplayTools::display(s, I, imageMworld, Xscale, Yscale);
    }
    display((const usNeedleModelBaseTip&)needleModel, I, imageMworld, Xscale, Yscale);
}
template void displayNeedle<unsigned char>(const usNeedleModelPolynomial&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, bool);
template void displayNeedle<vpRGBa>(const usNeedleModelPolynomial&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, bool);

template<class ImageDataType>
void displayBaseStaticTorsor(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    vpRotationMatrix R(imageMworld);
    vpTranslationVector T(imageMworld);
    
    vpColVector worldTorsor = -needleModel.getBaseStaticTorsor();
    vpColVector worldForce(worldTorsor, 0,3);
    vpColVector worldMoment(worldTorsor, 3,3);

    vpColVector imageBase = R * needleModel.getStartPoint() + T;
    vpColVector imageForce = R* worldForce;
    vpColVector imageMoment = R * worldMoment;

    double x = Xscale * imageBase[0];
    double y = Yscale * imageBase[1];

    double force_x = imageForce[0];
    double force_y = imageForce[1];

    double moment_x = imageMoment[0];
    double moment_y = imageMoment[1];

    vpDisplay::displayArrow(I, y, x, y+100*force_y, x+100*force_x, vpColor::red, 5, 5, 3);
    vpDisplay::displayArrow(I, y, x, y+3000*moment_y, x+3000*moment_x, vpColor::green, 5, 5, 3);
}
template void displayBaseStaticTorsor<unsigned char>(const usNeedleModelPolynomial&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayBaseStaticTorsor<vpRGBa>(const usNeedleModelPolynomial&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void displayCurvatureFromShape(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::displayCurvatureFromShape(needleModel, I, imageMworld, Xscale, Yscale);
}
template void displayCurvatureFromShape<unsigned char>(const usNeedleModelPolynomial&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayCurvatureFromShape<vpRGBa>(const usNeedleModelPolynomial&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void display(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, bool displayFullBody)
{
    displayNeedle(needleModel, I, imageMworld, Xscale, Yscale, displayFullBody);
    displayBaseStaticTorsor(needleModel, I, imageMworld, Xscale, Yscale);
    displayCurvatureFromShape(needleModel, I, imageMworld, Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleModelPolynomial&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, bool);
template void display<vpRGBa>(const usNeedleModelPolynomial&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, bool);

//! Display usNeedleModelSpline

template<class ImageDataType>
void displayNeedle(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, bool displayFullBody)
{
    usGeometryDisplayTools::displayLine((const usBSpline3D&)needleModel, I, imageMworld, Xscale, Yscale, vpColor::red);
    if(displayFullBody)
    {
        double r = 0.5 * needleModel.getOuterDiameter();
        vpColVector z(imageMworld.inverse().getCol(2));

        std::vector<double> lengths(needleModel.getNbSegments());
        std::vector<vpColVector> p1(needleModel.getNbSegments()+1);
        std::vector<vpColVector> p2(needleModel.getNbSegments()+1);
        for(int i=0 ; i<needleModel.getNbSegments() ; i++)
        {
            lengths.at(i) = needleModel.accessSegment(i).getParametricLength();
            vpColVector p(needleModel.accessSegment(i).getStartPoint());
            vpColVector x(vpColVector::crossProd(needleModel.accessSegment(i).getStartTangent(), z).normalize());
            p1.at(i) = p + r * x;
            p2.at(i) = p - r * x;
        }
        vpColVector p(needleModel.accessLastSegment().getEndPoint());
        vpColVector x(vpColVector::crossProd(needleModel.accessLastSegment().getEndTangent(), z).normalize());
        p1.back() = p + r * x;
        p2.back() = p - r * x;
        usBSpline3D s;
        s.defineFromPoints(p1, lengths, needleModel.accessSegment(0).getOrder());
        usGeometryDisplayTools::display(s, I, imageMworld, Xscale, Yscale);
        s.defineFromPoints(p2, lengths, needleModel.accessSegment(0).getOrder());
        usGeometryDisplayTools::display(s, I, imageMworld, Xscale, Yscale);
    }
    usGeometryDisplayTools::displayExtremities((const usBSpline3D&)needleModel, I, imageMworld, Xscale, Yscale, vpColor::red);
    display((const usNeedleModelBaseTip&)needleModel, I, imageMworld, Xscale, Yscale);
}
template void displayNeedle<unsigned char>(const usNeedleModelSpline&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, bool);
template void displayNeedle<vpRGBa>(const usNeedleModelSpline&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, bool);

template<class ImageDataType>
void displayBaseStaticTorsor(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    vpRotationMatrix R(imageMworld);
    vpTranslationVector T(imageMworld);
    
    vpColVector worldTorsor = -needleModel.getBaseStaticTorsor();
    vpColVector worldForce(worldTorsor, 0,3);
    vpColVector worldMoment(worldTorsor, 3,3);

    vpColVector imageBase = R * needleModel.accessSegment(0).getStartPoint() + T;
    vpColVector imageForce = R* worldForce;
    vpColVector imageMoment = R * worldMoment;

    double x = Xscale * imageBase[0];
    double y = Yscale * imageBase[1];

    double force_x = imageForce[0];
    double force_y = imageForce[1];

    double moment_x = imageMoment[0];
    double moment_y = imageMoment[1];

    vpDisplay::displayArrow(I, y, x, y+100*force_y, x+100*force_x, vpColor::red, 5, 5, 3);
    vpDisplay::displayArrow(I, y, x, y+3000*moment_y, x+3000*moment_x, vpColor::green, 5, 5, 3);
}
template void displayBaseStaticTorsor<unsigned char>(const usNeedleModelSpline &needleModel, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayBaseStaticTorsor<vpRGBa>(const usNeedleModelSpline &needleModel, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void displayCurvatureFromShape(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::displayCurvatureFromShape(needleModel, I, imageMworld, Xscale, Yscale);
}
template void displayCurvatureFromShape<unsigned char>(const usNeedleModelSpline&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayCurvatureFromShape<vpRGBa>(const usNeedleModelSpline&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void display(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, bool displayFullBody)
{
    displayNeedle(needleModel, I, imageMworld, Xscale, Yscale, displayFullBody);
    displayBaseStaticTorsor(needleModel, I, imageMworld, Xscale, Yscale);
    displayCurvatureFromShape(needleModel, I, imageMworld, Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleModelSpline&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, bool);
template void display<vpRGBa>(const usNeedleModelSpline&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, bool);

//! Display usVirtualSpring

template<class ImageType> 
void display(const usVirtualSpring &spring, const vpImage<ImageType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::display((const usOrientedPlane3D&)spring, I, imageMworld, Xscale, Yscale, vpColor::green);
}
template void display<unsigned char>(const usVirtualSpring&, const vpImage<unsigned char> &, const vpHomogeneousMatrix &, double, double);
template void display<vpRGBa>(const usVirtualSpring&, const vpImage<vpRGBa> &, const vpHomogeneousMatrix &, double, double);

//! Display usTissueModelSpline

template<class ImageDataType>
void display(const usTissueModelSpline &tissue, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::display(tissue.accessSurface(), I, imageMworld, Xscale, Yscale, vpColor::black);
    usGeometryDisplayTools::display(tissue.accessPath(), I, imageMworld, Xscale, Yscale, vpColor::blue);
}
template void display<unsigned char>(const usTissueModelSpline&, const vpImage<unsigned char> &, const vpHomogeneousMatrix &, double, double);
template void display<vpRGBa>(const usTissueModelSpline&, const vpImage<vpRGBa> &, const vpHomogeneousMatrix &, double, double);


//! Display usNeedleTip

template<class ImageDataType>
void display(const usNeedleTip &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::displayFrame(I, imageMworld * tip.getWorldMbase().inverse(), Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleTip&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleTip&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usNeedleTipActuated

template<class ImageDataType>
void display(const usNeedleTipActuated &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    vpRotationMatrix R(imageMworld);
    vpTranslationVector T(imageMworld);
    
    vpColVector b = tip.getBasePosition();
    vpColVector t = tip.getTipPosition();    
    vpColVector d = tip.getBaseAxisZ();
    
    vpColVector imageB(R * b + T);
    vpColVector imageT(R * t + T);
    vpColVector imageD(R * d);
    vpColVector n(2,0);
    n[0] = imageD[1];
    n[1] = -imageD[0];
    n.normalize();

    double diameter = tip.getDiameter();

    double x[3];
    double y[3];
    
    x[0] = Xscale*(imageT[0]);
    y[0] = Yscale*(imageT[1]);

    x[1] = Xscale*(imageB[0] + diameter/2*n[0]);
    y[1] = Yscale*(imageB[1] + diameter/2*n[1]);

    x[2] = Xscale*(imageB[0] - diameter/2*n[0]);
    y[2] = Yscale*(imageB[1] - diameter/2*n[1]);

    for(int i=0 ; i<3 ; i++)
    {
        vpDisplay::displayLine(I, y[i],x[i], y[(i+1)%3], x[(i+1)%3], vpColor::red);
    }
}
template void display<unsigned char>(const usNeedleTipActuated&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleTipActuated&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usNeedleTipBeveled

template<class ImageDataType>
void display(const usNeedleTipBeveled &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    vpRotationMatrix R(imageMworld);
    vpTranslationVector T(imageMworld);
    
    vpColVector b = tip.getBasePosition();
    vpColVector dy = tip.getBaseAxisY();
    vpColVector dz = tip.getBaseAxisZ();

    vpColVector imageB(R * b + T);
    vpColVector imageDy(R * dy);
    vpColVector imageDz(R * dz);
    
    double diameter = tip.getDiameter();
    double length = tip.getLength();
    
    double x[3];
    double y[3];
    
    x[0] = Xscale*(imageB[0] + diameter/2*imageDy[0]);
    y[0] = Yscale*(imageB[1] + diameter/2*imageDy[1]);

    x[1] = Xscale*(imageB[0] - diameter/2*imageDy[0]);
    y[1] = Yscale*(imageB[1] - diameter/2*imageDy[1]);

    x[2] = Xscale*(imageB[0] + diameter/2*imageDy[0] + length*imageDz[0]);
    y[2] = Yscale*(imageB[1] + diameter/2*imageDy[1] + length*imageDz[1]);

    for(int i=0 ; i<3 ; i++)
    {
        vpDisplay::displayLine(I, y[i],x[i], y[(i+1)%3], x[(i+1)%3], vpColor::red);
    }
}
template void display<unsigned char>(const usNeedleTipBeveled&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleTipBeveled&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usNeedleTipPrebent

template<class ImageDataType>
void display(const usNeedleTipPrebent &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    vpRotationMatrix R(imageMworld);
    vpTranslationVector T(imageMworld);
    
    vpColVector b = tip.getBasePosition();
    vpColVector t = tip.getTipPosition();    
    vpColVector d = tip.getBaseAxisZ();
    
    vpColVector imageB(R * b + T);
    vpColVector imageT(R * t + T);
    vpColVector imageD(R * d);
    vpColVector n(2,0);
    n[0] = imageD[1];
    n[1] = -imageD[0];
    n.normalize();

    double diameter = tip.getDiameter();
    
    double x[3];
    double y[3];

    x[0] = Xscale*(imageT[0]);
    y[0] = Yscale*(imageT[1]);

    x[1] = Xscale*(imageB[0] + diameter/2*n[0]);
    y[1] = Yscale*(imageB[1] + diameter/2*n[1]);

    x[2] = Xscale*(imageB[0] - diameter/2*n[0]);
    y[2] = Yscale*(imageB[1] - diameter/2*n[1]);

    for(int i=0 ; i<3 ; i++)
    {
        vpDisplay::displayLine(I, y[i],x[i], y[(i+1)%3], x[(i+1)%3], vpColor::red);
    }
}
template void display<unsigned char>(const usNeedleTipPrebent&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleTipPrebent&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usNeedleTipSymmetric

template<class ImageDataType>
void display(const usNeedleTipSymmetric &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    vpRotationMatrix R(imageMworld);
    vpTranslationVector T(imageMworld);
    
    vpColVector b = tip.getBasePosition();    
    vpColVector d = tip.getBaseAxisZ();
    
    vpColVector imageB(R * b + T);
    vpColVector imageD(R * d);
    vpColVector n(2,0);
    n[0] = imageD[1];
    n[1] = -imageD[0];
    n.normalize();
    
    double diameter = tip.getDiameter();
    double length = tip.getLength();
    
    double x[3];
    double y[3];
    
    x[0] = Xscale*(imageB[0] + length*imageD[0]);
    y[0] = Yscale*(imageB[1] + length*imageD[1]);

    x[1] = Xscale*(imageB[0] + diameter/2*n[0]);
    y[1] = Yscale*(imageB[1] + diameter/2*n[1]);

    x[2] = Xscale*(imageB[0] - diameter/2*n[0]);
    y[2] = Yscale*(imageB[1] - diameter/2*n[1]);

    for(int i=0 ; i<3 ; i++)
    {
        vpDisplay::displayLine(I, y[i],x[i], y[(i+1)%3], x[(i+1)%3], vpColor::red);
    }
}
template void display<unsigned char>(const usNeedleTipSymmetric&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleTipSymmetric&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usNeedleInsertionModelKinematic

template<class ImageDataType>
void display(const usNeedleInsertionModelKinematic &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    display(model.accessNeedle(), I, imageMworld, Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleInsertionModelKinematic&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleInsertionModelKinematic&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usNeedleInsertionModelRayleighRitzSpline

template<class ImageDataType>
void displayTissueLayers(const usNeedleInsertionModelRayleighRitzSpline &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    if(model.getNbLayers()<2 || model.accessTissue().accessSurface().getDirection().euclideanNorm()==0) return;

    if(model.accessTissue().accessPath().getNbSegments()>0)
    {
        double l = 0;
        double restIndex = 0;
        double nextLayerLength = model.getLayerLength(0);

        for(int i=1 ; i<model.getNbLayers() ; i++)
        {
            while(l+model.accessTissue().accessPath().accessSegment(restIndex).getParametricLength() < nextLayerLength && restIndex<model.accessTissue().accessPath().getNbSegments()-1)
            {
                l += model.accessTissue().accessPath().accessSegment(restIndex).getParametricLength();
                restIndex++;
            }

            double restParam = nextLayerLength-l;
            usOrientedPlane3D P(usGeometryTools::getNormalPlane(model.accessTissue().accessPath().accessSegment(restIndex), restParam));

            usGeometryDisplayTools::display(P, I, imageMworld, Xscale, Yscale, vpColor::black);

            nextLayerLength += model.getLayerLength(i);
        }
    }
    else
    {
        double nextLayerLength = model.getLayerLength(0);
        vpColVector p = model.accessTissue().accessSurface().getPosition();
        vpColVector d = model.accessTissue().accessSurface().getDirection();

        for(int i=1 ; i<model.getNbLayers() ; i++)
        {
            usOrientedPlane3D P(p + nextLayerLength *d, d);

            usGeometryDisplayTools::display(P, I, imageMworld, Xscale, Yscale, vpColor::black);

            nextLayerLength += model.getLayerLength(i);
        }
    }
}
template void displayTissueLayers<unsigned char>(const usNeedleInsertionModelRayleighRitzSpline&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayTissueLayers<vpRGBa>(const usNeedleInsertionModelRayleighRitzSpline&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void displayInteraction(const usNeedleInsertionModelRayleighRitzSpline &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    if(model.accessTissue().accessPath().getNbSegments()<1) return;

    double freeLength = model.getNeedleFreeLength();
    double length = model.accessNeedle().getFullLength();
    if(freeLength >= length) return;

    vpRotationMatrix R(imageMworld);
    vpTranslationVector T(imageMworld);
    
    vpColVector worldPoint(3);
    vpColVector worldRestPoint(3);
    vpColVector imagePoint(3);
    vpColVector imageRestPoint(3);

    for(int i=0 ; i<10 ; i++)
    {
        double l = freeLength + (length-freeLength)*(i/9.0);

        int restIndex = 0;
        double restParam = 0;
        if(!model.getCorrespondingPathPoint(l, restIndex, restParam)) continue;

        worldRestPoint = model.accessTissue().accessPath().accessSegment(restIndex).getPoint(restParam);
        worldPoint = model.accessNeedle().getPoint(l);

        imagePoint = R * worldPoint + T;
        imageRestPoint = R * worldRestPoint + T;

        double x = Xscale*imagePoint[0];
        double y = Yscale*imagePoint[1];

        double xr = Xscale*imageRestPoint[0];
        double yr = Yscale*imageRestPoint[1];

        vpDisplay::displayArrow(I, y,x,yr,xr,vpColor::black);
    }
}
template void displayInteraction<unsigned char>(const usNeedleInsertionModelRayleighRitzSpline&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayInteraction<vpRGBa>(const usNeedleInsertionModelRayleighRitzSpline&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

template<class ImageDataType>
void display(const usNeedleInsertionModelRayleighRitzSpline &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale, bool displayFullBody)
{
    display(model.accessNeedle(), I, imageMworld, Xscale, Yscale, displayFullBody);
    switch(model.getNeedleTipType())
    {
        case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::ActuatedTip:
        {
            display(dynamic_cast<const usNeedleTipActuated&>(model.accessNeedleTip()), I, imageMworld, Xscale, Yscale);
            break;
        }
        case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::BeveledTip:
        {
            display(dynamic_cast<const usNeedleTipBeveled&>(model.accessNeedleTip()), I, imageMworld, Xscale, Yscale);
            break;
        }
        case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::PrebentTip:
        {
            display(dynamic_cast<const usNeedleTipPrebent&>(model.accessNeedleTip()), I, imageMworld, Xscale, Yscale);
            break;
        }
        case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::SymmetricTip:
        {
            display(dynamic_cast<const usNeedleTipSymmetric&>(model.accessNeedleTip()), I, imageMworld, Xscale, Yscale);
            break;
        }
    }
    display(model.accessTissue(), I, imageMworld, Xscale, Yscale);
    displayTissueLayers(model, I, imageMworld, Xscale, Yscale);
    displayInteraction(model, I, imageMworld, Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleInsertionModelRayleighRitzSpline&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, bool);
template void display<vpRGBa>(const usNeedleInsertionModelRayleighRitzSpline&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, bool);

//! Display usNeedleInsertionModelVirtualSprings

template<class ImageDataType>
void display(const usNeedleInsertionModelVirtualSprings &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    for(int i=0 ; i<model.getNbSprings() ; i++)
    {
        if(model.accessSpring(i).IsPositionUpdateAllowed()) usGeometryDisplayTools::display((const usOrientedPlane3D&)model.accessSpring(i), I, imageMworld, Xscale, Yscale, vpColor::green);
        else usGeometryDisplayTools::display((const usOrientedPlane3D&)model.accessSpring(i), I, imageMworld, Xscale, Yscale, vpColor::red);
    }
    display(model.accessNeedle(), I, imageMworld, Xscale, Yscale);
    usGeometryDisplayTools::display(model.accessSurface(), I, imageMworld, Xscale, Yscale, vpColor::black);
    //displayBaseStaticTorsor(model.accessNeedle(), I, imageMworld, Xscale, Yscale);
    //displayCurvatureFromShape(model.accessNeedle(), I, imageMworld, Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleInsertionModelVirtualSprings&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleInsertionModelVirtualSprings&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);



} // namespace usNeedleModelingDisplayTools
