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
 * Pedro Patlan
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usRFToPreScan3DConverter.h
 * @brief 3D scan-converter
 */

#ifndef __usRFToPreScan3DConverter_h_
#define __usRFToPreScan3DConverter_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)
// external includes
#include <fftw3.h>

// std includes
#include <cmath>
#include <complex>
#include <vector>

// visp/ustk includes
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usLogCompressor.h>
#include <visp3/ustk_core/usRFToPreScan2DConverter.h>

/**
 * @class usRFToPreScan3DConverter
 * @brief 3D conversion from RF signal to pre-scan image
 * @ingroup module_ustk_core
 *
 * This class allows to convert 3D RF ultrasound images to pre-scan.
 * Here is an example to show how to use it :
 *
 * \code
#include <visp3/ustk_core/usRFToPreScan3DConverter.h>

int main()
{
  // example of 2D post-scan image settings
  unsigned int width = 320;
  unsigned int height = 240;
  unsigned int frames = 10;
  double transducerRadius = 0.045;
  double scanLinePitch = 0.0012;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double axialResolution = 0.002;
  double framePitch = 0.002;
  double motorRadius = 0.04;

  usImage3D<short int> I(height, width,frames);
  usImageRF3D<short int> rfImage; // to fill (image + settings)
  rfImage.setTransducerRadius(transducerRadius);
  rfImage.setScanLinePitch(scanLinePitch);
  rfImage.setScanLineNumber(scanLineNumber);
  rfImage.setTransducerConvexity(isTransducerConvex);
  rfImage.setAxialResolution(axialResolution);
  rfImage.setMotorRadius(motorRadius);
  rfImage.setMotorType(usMotorSettings::TiltingMotor);
  rfImage.setFramePitch(framePitch);

  usImagePreScan3D<unsigned char> prescanImage; // output
  usRFToPreScan3DConverter converter;
  converter.convert(rfImage,prescanImage);
}

 * \endcode
 */
class VISP_EXPORT usRFToPreScan3DConverter
{
public:
  usRFToPreScan3DConverter();

  ~usRFToPreScan3DConverter();

  void convert(const usImageRF3D<short int> &rfImage, usImagePreScan3D<unsigned char> &preScanImage);

  int getDecimationFactor();

  void setDecimationFactor(int decimationFactor);

protected:
  void init(int heightRF, int widthRF, int frameNumber);

private:
  usRFToPreScan2DConverter *m_converter;
  int m_frameNumber;
  int m_widthRF;
  int m_heightRF;
  bool m_isInit;
  int m_decimationFactor;
};

#endif // USTK_HAVE_FFTW
#endif // __usRFToPreScan3DConverter_h_
