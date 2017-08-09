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
 * @file usRFToPreScan3DConverter.cpp
 * @brief 3D RF to pre-scan
 */

#include <visp3/ustk_core/usRFToPreScan3DConverter.h>

#if defined(USTK_HAVE_FFTW)

/**
* Constructor.
* @param decimationFactor Decimation factor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
usRFToPreScan3DConverter::usRFToPreScan3DConverter(int decimationFactor) : m_converter(decimationFactor ) {

}

/**
* Destructor.
*/
usRFToPreScan3DConverter::~usRFToPreScan3DConverter() {

}

/**
* Convert method : performs the conversion from RF frame to a pre-scan frame using the following processes :
* - Enveloppe detector
* - Logarithmic compression
* - Decimation
*
* @param rfImage RF frame to convert
* @param preScanImage pre-scan image : result of convertion
*/
void usRFToPreScan3DConverter::convert(const usImageRF3D<short int> &rfImage, usImagePreScan3D<unsigned char> &preScanImage) {

  preScanImage.resize(rfImage.getDimX() ,rfImage.getDimY() / getDecimationFactor(),rfImage.getDimZ());

  // First we copy the transducer settings
  preScanImage.setImagePreScanSettings(rfImage);

  usImagePreScan2D<unsigned char> preScanFrame;
  usImageRF2D<short int> frameRF;

  //loop to convert each frame of the volume
  for(unsigned int i = 0; i<rfImage.getDimZ(); i++) {
    rfImage.getFrame(frameRF,i);
    m_converter.convert(frameRF, preScanFrame);
    preScanImage.insertFrame(preScanFrame,i);
  }
}

/**
* Decimation factor getter.
* @return Decimation factor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
int usRFToPreScan3DConverter::getDecimationFactor() {
  return m_converter.getDecimationFactor();
}

/**
* Decimation factor setter.
* @param  decimationFactor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
void usRFToPreScan3DConverter::setDecimationFactor(int decimationFactor) {
  m_converter.setDecimationFactor(decimationFactor);
}

#endif
