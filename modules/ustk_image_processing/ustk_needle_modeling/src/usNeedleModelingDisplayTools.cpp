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
void displayNeedle(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::display((const usPolynomialCurve3D&)needleModel, I, imageMworld, Xscale, Yscale, vpColor::red, 10);
    display((const usNeedleModelBaseTip&)needleModel, I, imageMworld, Xscale, Yscale);
}
template void displayNeedle<unsigned char>(const usNeedleModelPolynomial&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayNeedle<vpRGBa>(const usNeedleModelPolynomial&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

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
void display(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    displayNeedle(needleModel, I, imageMworld, Xscale, Yscale);
    displayBaseStaticTorsor(needleModel, I, imageMworld, Xscale, Yscale);
    displayCurvatureFromShape(needleModel, I, imageMworld, Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleModelPolynomial&, const vpImage<unsigned char>&, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale);
template void display<vpRGBa>(const usNeedleModelPolynomial&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale);

//! Display usNeedleModelSpline

template<class ImageDataType>
void displayNeedle(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::displayLine((const usBSpline3D&)needleModel, I, imageMworld, Xscale, Yscale, vpColor::red);
    usGeometryDisplayTools::displayExtremities((const usBSpline3D&)needleModel, I, imageMworld, Xscale, Yscale, vpColor::red);
    display((const usNeedleModelBaseTip&)needleModel, I, imageMworld, Xscale, Yscale);
}
template void displayNeedle<unsigned char>(const usNeedleModelSpline &needleModel, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void displayNeedle<vpRGBa>(const usNeedleModelSpline &needleModel, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

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
void display(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    displayNeedle(needleModel, I, imageMworld, Xscale, Yscale);
    displayBaseStaticTorsor(needleModel, I, imageMworld, Xscale, Yscale);
    displayCurvatureFromShape(needleModel, I, imageMworld, Xscale, Yscale);
}
template void display<unsigned char>(const usNeedleModelSpline&, const vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double);
template void display<vpRGBa>(const usNeedleModelSpline&, const vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double);

//! Display usVirtualSpring

template<class ImageType> 
void display(const usVirtualSpring &spring, const vpImage<ImageType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::display((const usOrientedPlane3D&)spring, I, imageMworld, Xscale, Yscale, vpColor::green);
}
template void display<unsigned char>(const usVirtualSpring&, const vpImage<unsigned char> &, const vpHomogeneousMatrix &, double, double);
template void display<vpRGBa>(const usVirtualSpring&, const vpImage<vpRGBa> &, const vpHomogeneousMatrix &, double, double);

//! Display usVirtualSpring

template<class ImageDataType>
void display(const usTissueModelSpline &tissue, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale, double Yscale)
{
    usGeometryDisplayTools::display(tissue.accessSurface(), I, imageMworld, Xscale, Yscale, vpColor::black);
    usGeometryDisplayTools::display(tissue.accessPath(), I, imageMworld, Xscale, Yscale, vpColor::blue);
}

} // namespace usNeedleModelingDisplayTools
