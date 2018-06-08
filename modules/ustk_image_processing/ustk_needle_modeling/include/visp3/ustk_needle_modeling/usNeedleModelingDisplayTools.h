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

#include <visp3/ustk_needle_modeling/usNeedleModelBaseTip.h>
#include <visp3/ustk_needle_modeling/usNeedleModelPolynomial.h>
#include <visp3/ustk_needle_modeling/usNeedleModelSpline.h>


namespace usNeedleModelingDisplayTools
{

//! Display usNeedleModelBaseTip

template<class ImageDataType>
VISP_EXPORT void displayBase(const usNeedleModelBaseTip &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void displayTip(const usNeedleModelBaseTip &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template <class ImageDataType>
VISP_EXPORT void display(const usNeedleModelBaseTip &needleModel, vpImage<unsigned char> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleModelPolynomial

template <class ImageDataType>
VISP_EXPORT void displayNeedle(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template <class ImageDataType>
VISP_EXPORT void displayBaseStaticTorsor(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template <class ImageDataType>
VISP_EXPORT void displayCurvatureFromShape(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template <class ImageDataType>
VISP_EXPORT void displayAll(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleModelSpline

template<class ImageDataType>
VISP_EXPORT void displayNeedle(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void displayBaseStaticTorsor(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void displayCurvatureFromShape(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void displayAll(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

} // namespace usNeedleModelingDisplayTools
