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
 * @file usRFToPostScan2DConverter.cpp
 * @brief 2D scan-converter
 */

#include <visp3/ustk_core/usRFToPostScan2DConverter.h>

#if defined(USTK_HAVE_FFTW)

/**
* Constructor.
* @param decimationFactor Decimation factor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
usRFToPostScan2DConverter::usRFToPostScan2DConverter(int decimationFactor) : m_RFConverter(decimationFactor),
  m_scanConverter() {
}

/**
* Destructor.
*/
usRFToPostScan2DConverter::~usRFToPostScan2DConverter() {

}

/**
* Convert method : performs the conversion from RF frame to a post-scan frame using the following processes :
* - Enveloppe detector
* - Logarithmic compression
* - Decimation
* - Scan conversion
*
* @param rfImage RF frame to convert
* @param postScanImage post-scan image : result of convertion
*/
void usRFToPostScan2DConverter::convert(const usImageRF2D<short int> &rfImage, usImagePostScan2D<unsigned char> &postScanImage) {
  usImagePreScan2D<unsigned char> preScanImage;
  m_RFConverter.convert(rfImage,preScanImage);
  m_scanConverter.run(preScanImage,postScanImage);
}

/**
* (Re-)Initialize the converter.
* @param inputSettings Post-scan settings : transducer radius, pitch, depth, and resolutions.
* @param BModeSampleNumber Number of samples along a scan line in pre-scan image.
* @param scanLineNumber Number of scan lines : width of the RF image to convert.
* @param decimationFactor Decimation factor : keep only 1 pre-scan sample every N sample from RF signal (N = decimationFactor).
* @warning Make sure RFsampleNumer / decimationFactor = BModeSampleNumber
*/
void usRFToPostScan2DConverter::setConversionParameters(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
                                                            const int scanLineNumber, const int decimationFactor) {
  m_scanConverter.init(inputSettings,BModeSampleNumber,scanLineNumber);
  m_RFConverter.setDecimationFactor(decimationFactor);
}
#endif
