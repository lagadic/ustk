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

#include <visp3/ustk_core/usImage3D.h>
#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>


int main(int argc, const char **argv)
{
  (void)argc;
  (void)argv;

  // Generate an example of 3D pre-scan image settings

  unsigned int scanLineNumber = 256;
  unsigned int sampleNumber = 240;
  unsigned int frameNumber = 10;
  double transducerRadius = 0.045;
  double scanLinePitch = 0.0012;
  bool isTransducerConvex = true;
  double axialResolution = 0.002;
  double framePitch = 10*M_PI/180;
  double motorRadius = 0.04;

  usImagePreScan3D <unsigned char> preScan;

  // Fill pre-scan volume with some data

  usImage3D<unsigned char> I(sampleNumber, scanLineNumber, frameNumber);
  for(unsigned int k=0 ; k<frameNumber ; k+=2 ) {
    for(unsigned int j=0 ; j<scanLineNumber ; j++ ) {
      for(unsigned int i=0 ; i<sampleNumber ; i++ ) {
        I(i,j,k,255);
      }
    }
  }
  preScan.setData(I);
  
  // Fill pre-scan volume info

  preScan.setTransducerRadius(transducerRadius);
  preScan.setScanLinePitch(scanLinePitch);
  preScan.setScanLineNumber(scanLineNumber);
  preScan.setTransducerConvexity(isTransducerConvex);
  preScan.setAxialResolution(axialResolution);
  preScan.setMotorRadius(motorRadius);
  preScan.setMotorType(usMotorSettings::TiltingMotor);
  preScan.setFramePitch(framePitch);

  // Test all conversion optimization methods and compare timings

  usImagePostScan3D<unsigned char> postscan[18];
  
  usPreScanToPostScan3DConverter::usConverterOptimizationMethod optMethod[9] = 
      {usPreScanToPostScan3DConverter::SINGLE_THREAD_DIRECT_CONVERSION,
       usPreScanToPostScan3DConverter::MULTI_THREAD_DIRECT_CONVERSION,
       usPreScanToPostScan3DConverter::GPU_DIRECT_CONVERSION,
       usPreScanToPostScan3DConverter::SINGLE_THREAD_REDUCED_LOOKUP_TABLE,
       usPreScanToPostScan3DConverter::MULTI_THREAD_REDUCED_LOOKUP_TABLE,
       usPreScanToPostScan3DConverter::GPU_REDUCED_LOOKUP_TABLE,
       usPreScanToPostScan3DConverter::SINGLE_THREAD_FULL_LOOKUP_TABLE,
       usPreScanToPostScan3DConverter::MULTI_THREAD_FULL_LOOKUP_TABLE,
       usPreScanToPostScan3DConverter::GPU_FULL_LOOKUP_TABLE};
  
  char method[9][35] = 
      {"SINGLE_THREAD_DIRECT_CONVERSION",
       "MULTI_THREAD_DIRECT_CONVERSION",
       "GPU_DIRECT_CONVERSION",
       "SINGLE_THREAD_REDUCED_LOOKUP_TABLE",
       "MULTI_THREAD_REDUCED_LOOKUP_TABLE",
       "GPU_REDUCED_LOOKUP_TABLE",
       "SINGLE_THREAD_FULL_LOOKUP_TABLE",
       "MULTI_THREAD_FULL_LOOKUP_TABLE",
       "GPU_FULL_LOOKUP_TABLE"};
      
  for(int i=0 ; i<9 ; i++)
  {
    std::cout << "---------- Optimization method: " << method[i] << std::endl;
    usPreScanToPostScan3DConverter converter;
    try
    {
      converter.setConverterOptimizationMethod(optMethod[i]);
      std::cout << "---- Converter initialization:" << std::endl;
      double t = vpTime::measureTimeMs();
      converter.init(preScan);
      std::cout << "Timing: " << vpTime::measureTimeMs()-t << std::endl;
      std::cout << "---- Volume conversion:" << std::endl;
      std::cout << "-- Forward wobbling:" << std::endl;
      converter.SweepInZdirection(true);
      t = vpTime::measureTimeMs();
      converter.convert(postscan[2*i], preScan);
      std::cout << "Timing: " << vpTime::measureTimeMs()-t <<  std::endl;
      std::cout << "-- Backward wobbling:" << std::endl;
      converter.SweepInZdirection(false);
      t = vpTime::measureTimeMs();
      converter.convert(postscan[2*i+1], preScan);
      std::cout << "Timing: " << vpTime::measureTimeMs()-t <<  std::endl;
      std::cout << "---- End of conversion" << std::endl;
    }
    catch(std::exception &e)
    {
      std::cout << e.what() << std::endl;
    }
  }  
  
  return 0;
}
