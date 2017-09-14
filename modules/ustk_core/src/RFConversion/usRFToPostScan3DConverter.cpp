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
 * @file usRFToPostScan3DConverter.cpp
 * @brief 3D RF to post-scan
 */

#include <visp3/ustk_core/usRFToPostScan3DConverter.h>

#if defined(USTK_HAVE_FFTW)

/**
* Constructor.
* @param decimationFactor Decimation factor for RF conversion (keeping 1 RF sample every decimationFactor samples)
*/
usRFToPostScan3DConverter::usRFToPostScan3DConverter(int decimationFactor) : m_RFConverter(), m_scanConverter(), m_scanConverterIsInit(false),
                                                         m_inputImageChanged(false), m_intermediateImage() {
  m_RFConverter.setDecimationFactor(decimationFactor);
}

/**
* Destructor.
*/
usRFToPostScan3DConverter::~usRFToPostScan3DConverter() {

}

/**
* Convert method : performs the conversion from RF frame to a post-scan volume.
*
* @param rfImage RF frame to convert
* @param postScanImage post-scan image : result of convertion. But make sure
*/
void usRFToPostScan3DConverter::convert(const usImageRF3D<short int> &rfImage, usImagePostScan3D<unsigned char> &postScanImage) {
  m_RFConverter.convert(rfImage,m_intermediateImage);

  // this init method checks if parameters are the same, to avoid recomputing all the init proccess if not necessarry
  m_scanConverter.init(m_intermediateImage);

  m_scanConverter.convert(postScanImage, m_intermediateImage);
}

#endif
