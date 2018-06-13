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

#ifndef __usNeedleModelingDisplayTools_h_
#define __usNeedleModelingDisplayTools_h_

#include <visp3/ustk_needle_detection/usGeometryDisplayTools.h>

#include <visp3/ustk_needle_modeling/usNeedleModelBaseTip.h>
#include <visp3/ustk_needle_modeling/usNeedleModelPolynomial.h>
#include <visp3/ustk_needle_modeling/usNeedleModelSpline.h>

#include <visp3/ustk_needle_modeling/usTissueModelSpline.h>
#include <visp3/ustk_needle_modeling/usVirtualSpring.h>

#include <visp3/ustk_needle_modeling/usNeedleTip.h>
#include <visp3/ustk_needle_modeling/usNeedleTipActuated.h>
#include <visp3/ustk_needle_modeling/usNeedleTipBeveled.h>
#include <visp3/ustk_needle_modeling/usNeedleTipPrebent.h>
#include <visp3/ustk_needle_modeling/usNeedleTipSymmetric.h>

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelKinematic.h>
#include <visp3/ustk_needle_modeling/usNeedleInsertionModelRayleighRitzSpline.h>
#include <visp3/ustk_needle_modeling/usNeedleInsertionModelVirtualSprings.h>


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
VISP_EXPORT void displayNeedle(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, bool displayFullBody = false);
template <class ImageDataType>
VISP_EXPORT void displayBaseStaticTorsor(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template <class ImageDataType>
VISP_EXPORT void displayCurvatureFromShape(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template <class ImageDataType>
VISP_EXPORT void display(const usNeedleModelPolynomial &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, bool displayFullBody = false);

//! Display usNeedleModelSpline

template<class ImageDataType>
VISP_EXPORT void displayNeedle(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, bool displayFullBody = false);
template<class ImageDataType>
VISP_EXPORT void displayBaseStaticTorsor(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void displayCurvatureFromShape(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void display(const usNeedleModelSpline &needleModel, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, bool displayFullBody = false);

//! Display usVirtualSpring

template<class ImageDataType>
VISP_EXPORT void display(const usVirtualSpring &spring, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usTissueModelSpline

template<class ImageDataType>
VISP_EXPORT void display(const usTissueModelSpline &tissue, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleTip

template<class ImageDataType>
VISP_EXPORT void display(const usNeedleTip &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleTipActuated

template<class ImageDataType>
VISP_EXPORT void display(const usNeedleTipActuated &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleTipBeveled

template<class ImageDataType>
VISP_EXPORT void display(const usNeedleTipBeveled &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleTipPrebent

template<class ImageDataType>
VISP_EXPORT void display(const usNeedleTipPrebent &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleTipSymmetric

template<class ImageDataType>
VISP_EXPORT void display(const usNeedleTipSymmetric &tip, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleInsertionModelKinematic

template<class ImageDataType>
VISP_EXPORT void display(const usNeedleInsertionModelKinematic &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);

//! Display usNeedleInsertionModelRayleighRitzSpline

template<class ImageDataType>
VISP_EXPORT void displayLayers(const usNeedleInsertionModelRayleighRitzSpline &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void displayInteraction(const usNeedleInsertionModelRayleighRitzSpline &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000);
template<class ImageDataType>
VISP_EXPORT void display(const usNeedleInsertionModelRayleighRitzSpline &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, bool displayFullBody = false);

//! Display usNeedleInsertionModelVirtualSprings

template<class ImageDataType>
VISP_EXPORT void display(const usNeedleInsertionModelVirtualSprings &model, const vpImage<ImageDataType> &I, const vpHomogeneousMatrix &imageMworld, double Xscale = 3000, double Yscale = 3000, bool displayFullBody = false);


} // namespace usNeedleModelingDisplayTools

#endif // __usNeedleModelingDisplayTools_h_
