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
 * Authors:
 * Jason Chevrie
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usPreScanToPostScan3DConverter.h
 * @brief 3D scan-converter
 */

#ifndef __usPreScanToPostScan3DConverter_h_
#define __usPreScanToPostScan3DConverter_h_

#include <cmath>
#include <vector>

#include <visp3/ustk_core/usConfig.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>

/**
 * @class usPreScanToPostScan3DConverter
 * @brief 3D scan converter
 * @ingroup module_ustk_core
 *
 * This class allows to convert 3D pre-scan ultrasound images to post-scan.
 * The converter can be initialized through init() and then applied through convert().
 * This class accepts only images acquired by a convex transducer and a tilting motor for now.
 * The optimization method used to perform the conversion can be set through setConverterOptimizationMethod() before the call to init().
 * 
 * @warning Converting with *_REDUCED_LOOKUP_TABLE or *_FULL_LOOKUP_TABLE optimizations method can use a lot of RAM when computing the LUTs in init().
 * @warning Converting with *_DIRECT_CONVERSION optimization methods can lead to long conversion time.
 * 
 * Considering the following usImagePreScan3D image as input:
 * \image html img-usImagePreScan3D.png
 * this class generates an usImagePostScan3D (convex or linear) as output:
 * \image html img-usImagePostScan3D-linear.png
 * \image html img-usImagePostScan3D-convex.png
 *
 *  Here is an example of how to use this converter :
 *
 * \code
#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>

int main()
{
  // example of 3D pre-scan image settings
  unsigned int width = 320;
  unsigned int height = 240;
  unsigned int frames = 10;
  double transducerRadius = 0.045;
  double scanLinePitch = 0.0012;
  unsigned int scanLineNumber = width;
  bool isTransducerConvex = true;
  double axialResolution = 0.002;
  double framePitch = 0.002;
  double motorRadius = 0.04;

  usImage3D<unsigned char> I(height, width, frames);
  usImagePreScan3D <unsigned char> preScan; // your input pre-scan image
  // then you can fill the preScan image and settings
  preScan.setTransducerRadius(transducerRadius);
  preScan.setScanLinePitch(scanLinePitch);
  preScan.setScanLineNumber(scanLineNumber);
  preScan.setTransducerConvexity(isTransducerConvex);
  preScan.setAxialResolution(axialResolution);
  preScan.setMotorRadius(motorRadius);
  preScan.setMotorType(usMotorSettings::TiltingMotor);
  preScan.setFramePitch(framePitch);

  preScan.setData(I);

  usImagePostScan3D<unsigned char> postscanImage;
  //scan-converster
  usPreScanToPostScan3DConverter converter;
  converter.init(preScan);
  converter.convert(postscanImage, preScan);
}
 *  \endcode
 */
class VISP_EXPORT usPreScanToPostScan3DConverter
{
public:
  typedef enum {
    SINGLE_THREAD_DIRECT_CONVERSION,
    MULTI_THREAD_DIRECT_CONVERSION,
    GPU_DIRECT_CONVERSION,
    SINGLE_THREAD_FULL_LOOKUP_TABLE,
    MULTI_THREAD_FULL_LOOKUP_TABLE,
    GPU_FULL_LOOKUP_TABLE,
    SINGLE_THREAD_REDUCED_LOOKUP_TABLE,
    MULTI_THREAD_REDUCED_LOOKUP_TABLE,
    GPU_REDUCED_LOOKUP_TABLE
  } usConverterOptimizationMethod;
protected:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  class usVoxelWeightAndIndex
  {
    friend class usPreScanToPostScan3DConverter;
    unsigned int m_outputIndex;
    unsigned int m_inputIndex[8];
    double m_W[8];
  };
  class usVoxelWeightAndIndexReducedMemory
  {
      friend class usPreScanToPostScan3DConverter;
      unsigned int m_outputIndex;
      unsigned int m_inputIndex;
      double m_W[3];
  };
#endif
  
  usConverterOptimizationMethod m_converterOptimizationMethod;
  usConverterOptimizationMethod m_conversionOptimizationMethodUsedAtInit;

  std::vector<usVoxelWeightAndIndex> m_lookupTables[2];
  std::vector<usVoxelWeightAndIndexReducedMemory> m_reducedLookupTables[2];

  usImagePreScan3D<unsigned char> m_VpreScan;

  double m_downSamplingFactor;
  double m_resolution;
  bool m_SweepInZdirection;

  unsigned int m_nbX;
  unsigned int m_nbY;
  unsigned int m_nbZ;

  bool m_initDone;

public:
  usPreScanToPostScan3DConverter();
  usPreScanToPostScan3DConverter(const usImagePreScan3D<unsigned char> &preScanImage, double down);
  virtual ~usPreScanToPostScan3DConverter();

  void convert(usImagePostScan3D<unsigned char> &postScanImage, const usImagePreScan3D<unsigned char> &preScanImage);

  void init(const usImagePreScan3D<unsigned char> &preScanImage, double down = 1);

  void setConverterOptimizationMethod(usConverterOptimizationMethod method);
  usConverterOptimizationMethod setConverterOptimizationMethod() const {return m_converterOptimizationMethod;}
  
  double getResolution() const { return m_resolution; }

  void SweepInZdirection(bool flag) { m_SweepInZdirection = flag; }

private:
  void convertPreScanCoordToPostScanCoord(double i_preScan, double j_preScan, double k_preScan,
                                          double *i_postScan = NULL, double *j_postScan = NULL,
                                          double *k_postScan = NULL, bool sweepInZdirection = true);
  void convertPostScanCoordToPreScanCoord(double x, double y, double z, double *i = NULL, double *j = NULL,
                                          double *k = NULL, bool sweepInZdirection = true);
};

#endif // __usPreScanToPostScan3DConverter_h_
