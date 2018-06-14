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
* @file usRectangleVisualServoingController.cpp
* @brief Wrapper aroud usScanlineConfidence2D to control a robot using with Qt sigals/slots mecanism.
*/

#include <visp3/ustk_gui/usRectangleVisualServoingController.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_TEMPLATE_TRACKING)

/**
* Constructor.
*/
usRectangleVisualServoingController::usRectangleVisualServoingController(QObject *parent): QObject(parent), m_tracker(),
m_confidenceProcess(),m_converter(),m_imagePreScan(),m_confidencePreScan(),m_imagePostScan(),m_trackerActivated(false) ,m_controllerActivated(false)  {}

usRectangleVisualServoingController::~usRectangleVisualServoingController() {}

void usRectangleVisualServoingController::updateImage(usImagePreScan2D<unsigned char> image) {

  m_imagePreScan = image;

  // convert to post-scan
  m_converter.convert(m_imagePreScan,m_imagePostScan);
  emit(newPostScanFrame(m_imagePostScan));
  qApp->processEvents();

  // run the tracking process and get the result
  if(m_tracker.isInit()) {

    double xtarget, ytarget;

    // run confidence map process
    m_confidenceProcess.run(m_confidencePreScan, m_imagePreScan);
    m_tracker.update(m_imagePostScan);
    vpRectOriented rectangle = m_tracker.getTarget();

    if(m_trackerActivated)
      emit(newRectTracked(rectangle));

    usPixelMeterConversion::convert(m_imagePostScan, rectangle.getCenter().get_j(), rectangle.getCenter().get_i(), xtarget, ytarget);

    double ttarget = atan2(xtarget, ytarget);

    unsigned int height(m_confidencePreScan.getHeight()), width(m_confidencePreScan.getWidth());

    double I_sum = 0.0;
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j)
        I_sum += static_cast<double>(m_confidencePreScan[i][j]);

    double yc = 0.0;
    for (unsigned int x = 0; x < height; ++x)
      for (unsigned int y = 0; y < width; ++y) {
        yc += y * m_confidencePreScan[x][y];
      }
    yc /= I_sum;

    double tc = yc * m_confidencePreScan.getScanLinePitch() - m_confidencePreScan.getFieldOfView() / 2.0;

    double lambda_t = 1.2;
    double lambda_c = 0.8;

    // computing velocities
    double xControlVelocity = -lambda_t * xtarget + lambda_c * (tc - ttarget) * ytarget; // meters per second
    double thetaZControlVelocity = lambda_c * (tc - ttarget);// radians per second

    // send command
    if(m_controllerActivated) {
      emit(updateProbeZOrientation(vpMath::deg(thetaZControlVelocity) * 10)); // 10-1 deg per second
      emit(updatePobeXVelocity(xControlVelocity * 1000)); // mm per second
    }
  }
}

void usRectangleVisualServoingController::initTracker(vpRectOriented rect) {
  if(m_imagePostScan.getSize()!=0)
    m_tracker.init(m_imagePostScan,rect);
  else
    throw(vpException(vpException::fatalError,"No frames sent to controller, cannot init tracker"));
  m_trackerActivated = true;
}

void usRectangleVisualServoingController::activateController(bool activate)
{
  m_controllerActivated = activate;
  if(!activate) {
    emit(updateProbeZOrientation(0));
    emit(updatePobeXVelocity(0));
  }
}

void usRectangleVisualServoingController::activateController()
{
  activateController(true);
}

void usRectangleVisualServoingController::disactivateController()
{
  activateController(false);
}

void usRectangleVisualServoingController::stopTracking() {
  m_trackerActivated = false;
  m_controllerActivated = false;
  emit(updateProbeZOrientation(0));
  emit(updatePobeXVelocity(0));
}
#endif
