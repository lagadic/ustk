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

#ifndef __usGeometryDisplayTools_h_
#define __usGeometryDisplayTools_h_

#include <limits>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>

#include <visp3/ustk_needle_detection/usOrientedPlane3D.h>
#include <visp3/ustk_needle_detection/usPolynomialCurve2D.h>
#include <visp3/ustk_needle_detection/usPolynomialCurve3D.h>
#include <visp3/ustk_needle_detection/usBSpline3D.h>


namespace usGeometryDisplayTools
{

template<class ImageDataType>
VISP_EXPORT void displayFrame(const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMFrame, double Xscale = 3000, double Yscale = 3000);

//! Display usOrientedPlane3D

template <class ImageDataType>
VISP_EXPORT void display(const usOrientedPlane3D &plane, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, const vpColor &color = vpColor::green);

//! Display usPolynomialCurve2D

template<class ImageDataType>
VISP_EXPORT void display(const usPolynomialCurve2D &curve, const vpImage<ImageDataType> &I, double Xscale = 3000, double Yscale =3000, const vpColor &color = vpColor::red, int nbRenderingLines = 10);

//! Display usPolynomialCurve3D

template<class ImageDataType>
VISP_EXPORT void display(const usPolynomialCurve3D &curve, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale =3000, const vpColor &color = vpColor::red, int nbRenderingLines = 10, double visibilityDistance=std::numeric_limits<double>::infinity());
template<class ImageDataType>
VISP_EXPORT void displayCurvatureFromShape(const usPolynomialCurve3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, const vpColor &color = vpColor::black);

//! Display usBSpline3D

template<class ImageDataType>
VISP_EXPORT void displayLine(const usBSpline3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, const vpColor &color = vpColor::red, int nbRenderingLinesPerSegment = 10, double visibilityDistance=std::numeric_limits<double>::infinity());
template<class ImageDataType>
VISP_EXPORT void displayExtremities(const usBSpline3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, const vpColor &color = vpColor::red, double visibilityDistance=std::numeric_limits<double>::infinity());
template<class ImageDataType>
VISP_EXPORT void displayCurvatureFromShape(const usBSpline3D &spline, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, const vpColor &color = vpColor::black);

} // namespace usGeometryDisplayTools

#endif // __usGeometryDisplayTools_h_
