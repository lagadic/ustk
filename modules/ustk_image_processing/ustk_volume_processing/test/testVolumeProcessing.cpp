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
 * This file is provided AS IS with NO WARRANTJ OF ANJ KIND, INCLUDING THE
 * WARRANTJ OF DESIGN, MERCHANTABILITJ AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

/*!
  \example testVolumeProssessing.cpp

  USTK volume processing test

  This example tests all functions declared in the usVolumeProcessing class.
*/

#include <visp3/ustk_volume_processing/usVolumeProcessing.h>

#include <visp3/ustk_core/usImage3D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>


int main()
{
    const unsigned int size = 100;
    usImage3D<unsigned char> V1(size,size,size);
    usImage3D<unsigned char> V2(size,size,size);
    
    usVolumeProcessing::max(V1);
    usVolumeProcessing::min(V1);
    
    usImage3D<double> filterI = usVolumeProcessing::generateGaussianDerivativeFilterI(1,5);
    usImage3D<double> filterJ = usVolumeProcessing::generateGaussianDerivativeFilterJ(1,5);
    usImage3D<double> filterK = usVolumeProcessing::generateGaussianDerivativeFilterK(1,5);
    usImage3D<double> filterII = usVolumeProcessing::generateGaussianDerivativeFilterII(1,5);
    usImage3D<double> filterJJ = usVolumeProcessing::generateGaussianDerivativeFilterJJ(1,5);
    usImage3D<double> filterKK = usVolumeProcessing::generateGaussianDerivativeFilterKK(1,5);
    usImage3D<double> filterIJ = usVolumeProcessing::generateGaussianDerivativeFilterIJ(1,5);
    usImage3D<double> filterIK = usVolumeProcessing::generateGaussianDerivativeFilterIK(1,5);
    usImage3D<double> filterJK = usVolumeProcessing::generateGaussianDerivativeFilterJK(1,5);
    
    usVolumeProcessing::applyFilter(V1, filterI, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterJ, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterII, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterJJ, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterKK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterIJ, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterIK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::applyFilter(V1, filterJK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);

    usImage3D<double> Vfiltered;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterI);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterJ);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterK);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterII);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterJJ);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterKK);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterIJ);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterIK);
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterJK);

    usVolumeProcessing::derivativeI(V1, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::derivativeJ(V1, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    usVolumeProcessing::derivativeK(V1, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    
    usVolumeProcessing::derivativeI(V1, Vfiltered);
    usVolumeProcessing::derivativeJ(V1, Vfiltered);
    usVolumeProcessing::derivativeK(V1, Vfiltered);
    
    usVolumeProcessing::gaussianDerivativeI(V1, Vfiltered, 1, 5);
    usVolumeProcessing::gaussianDerivativeJ(V1, Vfiltered, 1, 5);
    usVolumeProcessing::gaussianDerivativeK(V1, Vfiltered, 1, 5);

    usImage3D<vpColVector> Vgrad;
    usVolumeProcessing::gradient(V1, Vgrad);
    
    usImage3D<vpMatrix> Vhes;
    usVolumeProcessing::hessian(V1, Vhes);
    
    usVolumeProcessing::frangi(V1, Vfiltered, 1, 1, 1);

    usImage3D<vpColVector> Vvect(size,size,size, vpColVector(3,1));
    usImage3D<double> Vnorm(size,size,size, 0);
    
    usVolumeProcessing::norm(Vvect, Vnorm);

    usImage3D<int> Vdiff;
    usVolumeProcessing::difference(V1, V2, Vdiff);
    usVolumeProcessing::absoluteDifference(V1, V2, Vdiff);
    
    vpColVector bar(3,0);
    usVolumeProcessing::computeBarycenter(V1, bar[0], bar[1], bar[2]);
    
    return 0;
}

