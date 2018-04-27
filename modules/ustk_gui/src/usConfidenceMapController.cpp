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
* @file usConfidenceMapController.cpp
* @brief Wrapper aroud usScanlineConfidence2D to control a robot using with Qt sigals/slots mecanism.
*/

#include <visp3/ustk_gui/usConfidenceMapController.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_CONFIDENCE_MAP)

/**
* Constructor.
*/
usConfidenceMapController::usConfidenceMapController(QObject *parent): QObject(parent), m_confidenceProcessor(),m_confidenceMap(), m_gain(5)  {}

usConfidenceMapController::~usConfidenceMapController() {}

void usConfidenceMapController::updateImage(usImagePreScan2D<unsigned char> image) {
  m_confidenceProcessor.run(m_confidenceMap,image);

  //robot orientation control law

  // confidence barycenter
  double I_sum = 0.0;
  for (unsigned int i = 0; i < m_confidenceMap.getHeight(); ++i)
    for (unsigned int j = 0; j < m_confidenceMap.getWidth(); ++j)
      I_sum += static_cast<double>(m_confidenceMap[i][j]);

  double yc = 0.0;
  for (unsigned int x = 0; x < m_confidenceMap.getHeight(); ++x)
    for (unsigned int y = 0; y < m_confidenceMap.getWidth(); ++y) {
      yc += y * m_confidenceMap[x][y];
    }
  yc /= I_sum;

  // compute error angle (taget = barycenter on the central scanline)
  double thetaError = yc * m_confidenceMap.getScanLinePitch() - m_confidenceMap.getFieldOfView() / 2.0;

  //proportionnal control (unit 10-1 deg per second)
  emit(updateProbeOrientation(vpMath::deg(m_gain * thetaError) * 10));
}

void  usConfidenceMapController::setPropotionnalControlGain(const double gain) {
  m_gain=gain;
}

double usConfidenceMapController::getPropotionnalControlGain() const {
  return m_gain;
}

#endif
