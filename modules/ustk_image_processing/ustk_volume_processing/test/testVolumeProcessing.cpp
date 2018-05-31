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
    std::cout << "Start testVolumeProcessing" << std::endl;
    const unsigned int size = 100;
    usImage3D<unsigned char> Vdefault;
    std::cout << "done: usImage3D<unsigned char> default contructor" << std::endl;
    usImage3D<unsigned char> V1(size,size,size);
    std::cout << "done: usImage3D<unsigned char> contructor with sizes" << std::endl;
    usImage3D<unsigned char> Vinit(size,size,size, 50);
    std::cout << "done: usImage3D<unsigned char> contructor with sizes and initialization" << std::endl;
    usImage3D<unsigned char> V2(V1, true);
    std::cout << "done: usImage3D<unsigned char> copy constructor" << std::endl;
    
    usVolumeProcessing::max(V1);
    std::cout << "done: usVolumeProcessing::max" << std::endl;
    usVolumeProcessing::min(V1);
    std::cout << "done: usVolumeProcessing::min" << std::endl;
    
    usImage3D<double> filterI = usVolumeProcessing::generateGaussianDerivativeFilterI(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterI" << std::endl;
    usImage3D<double> filterJ = usVolumeProcessing::generateGaussianDerivativeFilterJ(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterJ" << std::endl;
    usImage3D<double> filterK = usVolumeProcessing::generateGaussianDerivativeFilterK(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterK" << std::endl;
    usImage3D<double> filterII = usVolumeProcessing::generateGaussianDerivativeFilterII(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterII" << std::endl;
    usImage3D<double> filterJJ = usVolumeProcessing::generateGaussianDerivativeFilterJJ(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterJJ" << std::endl;
    usImage3D<double> filterKK = usVolumeProcessing::generateGaussianDerivativeFilterKK(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterKK" << std::endl;
    usImage3D<double> filterIJ = usVolumeProcessing::generateGaussianDerivativeFilterIJ(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterIJ" << std::endl;
    usImage3D<double> filterIK = usVolumeProcessing::generateGaussianDerivativeFilterIK(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterIK" << std::endl;
    usImage3D<double> filterJK = usVolumeProcessing::generateGaussianDerivativeFilterJK(1,5);
    std::cout << "done: usVolumeProcessing::generateGaussianDerivativeFilterJK" << std::endl;
    
    usVolumeProcessing::applyFilter(V1, filterI, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterI" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterJ, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterJ" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterK" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterII, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterII" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterJJ, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterJJ" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterKK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterKK" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterIJ, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterIJ" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterIK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterIK" << std::endl;
    usVolumeProcessing::applyFilter(V1, filterJK, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::applyFilter to voxel with filterJK" << std::endl;

    usImage3D<double> Vfiltered;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterI);
    std::cout << "done: usVolumeProcessing::applyFilter filterI" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterJ);
    std::cout << "done: usVolumeProcessing::applyFilter filterJ" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterK);
    std::cout << "done: usVolumeProcessing::applyFilter filterK" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterII);
    std::cout << "done: usVolumeProcessing::applyFilter filterII" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterJJ);
    std::cout << "done: usVolumeProcessing::applyFilter filterJJ" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterKK);
    std::cout << "done: usVolumeProcessing::applyFilter filterKK" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterIJ);
    std::cout << "done: usVolumeProcessing::applyFilter filterIJ" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterIK);
    std::cout << "done: usVolumeProcessing::applyFilter filterIK" << std::endl;
    usVolumeProcessing::applyFilter(V1, Vfiltered, filterJK);
    std::cout << "done: usVolumeProcessing::applyFilter filterJK" << std::endl;

    usVolumeProcessing::derivativeI(V1, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::derivativeI to voxel" << std::endl;
    usVolumeProcessing::derivativeJ(V1, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::derivativeJ to voxel" << std::endl;
    usVolumeProcessing::derivativeK(V1, V1.getHeight()/2, V1.getWidth()/2, V1.getNumberOfFrames()/2);
    std::cout << "done: usVolumeProcessing::derivativeK to voxel" << std::endl;
    
    usVolumeProcessing::derivativeI(V1, Vfiltered);
    std::cout << "done: usVolumeProcessing::derivativeI" << std::endl;
    usVolumeProcessing::derivativeJ(V1, Vfiltered);
    std::cout << "done: usVolumeProcessing::derivativeJ" << std::endl;
    usVolumeProcessing::derivativeK(V1, Vfiltered);
    std::cout << "done: usVolumeProcessing::derivativeK" << std::endl;
    
    usVolumeProcessing::gaussianDerivativeI(V1, Vfiltered, 1, 5);
    std::cout << "done: usVolumeProcessing::gaussianDerivativeI" << std::endl;
    usVolumeProcessing::gaussianDerivativeJ(V1, Vfiltered, 1, 5);
    std::cout << "done: usVolumeProcessing::gaussianDerivativeJ" << std::endl;
    usVolumeProcessing::gaussianDerivativeK(V1, Vfiltered, 1, 5);
    std::cout << "done: usVolumeProcessing::gaussianDerivativeK" << std::endl;

    usImage3D<vpColVector> Vgrad;
    usVolumeProcessing::gradient(V1, Vgrad);
    std::cout << "done: usVolumeProcessing::gradient" << std::endl;
    
    usImage3D<vpMatrix> Vhes;
    usVolumeProcessing::hessian(V1, Vhes);
    std::cout << "done: usVolumeProcessing::hessian" << std::endl;
#ifdef VISP_HAVE_GSL
    usVolumeProcessing::frangi(V1, Vfiltered, 1, 1, 1);
    std::cout << "done: usVolumeProcessing::frangi" << std::endl;
#else
    std::cout << "done: ViSP does not have GSL 3rd party to compute usVolumeProcessing::frangi" << std::endl;
#endif

    usImage3D<vpColVector> Vvect(size,size,size, vpColVector(3,1));
    std::cout << "done: usImage3D<vpColVector> contructor with sizes and initialization" << std::endl;
    usImage3D<double> Vnorm(size,size,size, 0);
    
    usVolumeProcessing::norm(Vvect, Vnorm);
    std::cout << "done: usVolumeProcessing::norm" << std::endl;

    usImage3D<int> Vdiff;
    std::cout << "done: usImage3D<int> default contructor" << std::endl;
    usVolumeProcessing::difference(V1, V2, Vdiff);
    std::cout << "done: usVolumeProcessing::difference" << std::endl;
    usVolumeProcessing::absoluteDifference(V1, V2, Vdiff);
    std::cout << "done: usVolumeProcessing::absoluteDifference" << std::endl;
    
    vpColVector bar(3,0);
    usVolumeProcessing::computeBarycenter(V1, bar[0], bar[1], bar[2]);
    std::cout << "done: usVolumeProcessing::computeBarycenter" << std::endl;
    
    return 0;
}

