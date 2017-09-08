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

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

/**
* Constructor.
* @param decimationFactor Decimation factor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
usRFToPreScan3DConverter::usRFToPreScan3DConverter() :m_frameNumber(), m_isInit(false), m_decimationFactor(10) {

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
  if(!m_isInit || (((int)rfImage.getDimZ()) != m_frameNumber)
     || (((int)rfImage.getDimY()) != m_heightRF)|| (((int)rfImage.getDimX()) != m_widthRF)) {
    init(rfImage.getDimY(),rfImage.getDimX(),rfImage.getDimZ());
  }
  preScanImage.resize(rfImage.getDimX() ,rfImage.getDimY() / getDecimationFactor(),rfImage.getDimZ());
  // First we copy the transducer/motor settings
  preScanImage.setImagePreScanSettings(rfImage);
  preScanImage.setAxialResolution(rfImage.getDepth() / preScanImage.getDimY());
  preScanImage.setMotorSettings(rfImage);
  std::vector<usImagePreScan2D<unsigned char> > preScanFrame;
  preScanFrame.resize(m_frameNumber);
  std::vector<usImageRF2D<short int> > frameRF;
  frameRF.resize(m_frameNumber);
  //loop to convert each frame of the volume

  for(int i = 0; i<m_frameNumber; i++) {
    rfImage.getFrame(frameRF.at(i),i);
  }

  if(!m_isInit) {
    init(rfImage.getDimY(),rfImage.getDimZ(),rfImage.getDimZ());
  }

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for(int i = 0; i<m_frameNumber; i++) {
    m_converter[i].convert(frameRF[i], preScanFrame.at(i));
  }

  for(int i = 0; i<m_frameNumber; i++) {
    preScanImage.insertFrame(preScanFrame[i],i);
  }
}

/**
* Decimation factor getter.
* @return Decimation factor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
int usRFToPreScan3DConverter::getDecimationFactor() {
  return m_decimationFactor;
}

/**
* Decimation factor setter.
* @param  decimationFactor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
void usRFToPreScan3DConverter::setDecimationFactor(int decimationFactor) {
  for(int i=0; i<m_frameNumber; i++)
    m_converter[i].setDecimationFactor(decimationFactor);
  m_decimationFactor = decimationFactor;
}

/**
* Initialisation of the converter.
*/
void usRFToPreScan3DConverter::init(int heightRF, int widthRF, int frameNumber) {
  //first init
  if(!m_isInit) {
    m_converter = new usRFToPreScan2DConverter[frameNumber];
    for(int i=0; i<frameNumber; i++)
      m_converter[i].init(widthRF,heightRF);

    m_isInit = true;
    m_frameNumber = frameNumber;
    m_heightRF = heightRF;
    m_widthRF = widthRF;
  }
  //update image size
  else if (m_frameNumber != frameNumber || m_heightRF != heightRF || m_widthRF != widthRF) {
    for(int i=0; i<m_frameNumber; i++)
      delete &(m_converter[i]);

    m_frameNumber = frameNumber;
    m_heightRF = heightRF;
    m_widthRF = widthRF;
    m_converter = new usRFToPreScan2DConverter[m_frameNumber];
    for(int i=0; i<m_frameNumber; i++)
      m_converter[i].init(m_widthRF,m_heightRF);
  }
}
#endif
