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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usRFToPostScan3DConverter.h
 * @brief 3D converter from RF to post-scan.
 */

#ifndef __usRFToPostScan3DConverter_h_
#define __usRFToPostScan3DConverter_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

// visp/ustk includes
#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>
#include <visp3/ustk_core/usRFToPreScan3DConverter.h>

/**
 * @class usRFToPostScan3DConverter
 * @brief 3D conversion from RF signal to post-scan image
 * @ingroup module_ustk_core
 *
 * This class allows to convert 3D RF ultrasound images to post-scan.
 * Here is an example to show how to use it :
 *
 * \code
#include <visp3/ustk_core/usRFToPostScan3DConverter.h>

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

  rfImage.setData(I);

  usImagePostScan3D<unsigned char> postscanImage; // output
  usRFToPostScan3DConverter converter;
  converter.convert(rfImage,postscanImage);
}
 * \endcode
 */
class VISP_EXPORT usRFToPostScan3DConverter
{
public:
  usRFToPostScan3DConverter(int decimationFactor = 10);

  ~usRFToPostScan3DConverter();

  void convert(const usImageRF3D<short int> &rfImage, usImagePostScan3D<unsigned char> &postScanImage);

private:
  usRFToPreScan3DConverter m_RFConverter;
  usPreScanToPostScan3DConverter m_scanConverter;

  usImagePreScan3D<unsigned char> m_intermediateImage;
};

#endif // USTK_HAVE_FFTW
#endif // __usRFToPostScan3DConverter_h_
