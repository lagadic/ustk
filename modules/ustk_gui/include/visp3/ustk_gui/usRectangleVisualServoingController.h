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
 *****************************************************************************/

/**
 * @file usRectangleVisualServoingController.h
 * @brief Qt wrapper for a controller based on confidence map and dense tracker.
 */

#ifndef __usRectangleVisualServoingController_h_
#define __usRectangleVisualServoingController_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_TEMPLATE_TRACKING)

#include <QApplication>
#include <QObject>
#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usPixelMeterConversion.h>
#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>
#include <visp3/ustk_template_tracking/usDenseTracker2D.h>

/**
 * @class usRectangleVisualServoingController
 * @brief Qt wrapper for a controller based on confidence map and dense tracker.
 * @ingroup module_ustk_gui
 */
class VISP_EXPORT usRectangleVisualServoingController : public QObject
{
  Q_OBJECT
public:
  usRectangleVisualServoingController(QObject *parent = NULL);
  virtual ~usRectangleVisualServoingController();

signals:
  void confidenceBarycenterAngle(double theta);
  void confidenceMap(usImagePreScan2D<unsigned char>);
  void updatePobeXVelocity(int linearVelocity);
  void updateProbeZOrientation(int angularVelocity);
  void newRectTracked(vpRectOriented rect);
  void newPostScanFrame(usImagePostScan2D<unsigned char> image);
  void trackerXError(double error);

public slots:
  void updateImage(usImagePreScan2D<unsigned char> image);
  void initTracker(vpRectOriented rect);
  void stopTracking();
  void activateController(bool activate);
  void activateController();
  void disactivateController();

private:
  usDenseTracker2D m_tracker;
  usScanlineConfidence2D m_confidenceProcess;
  usPreScanToPostScan2DConverter m_converter;
  usImagePreScan2D<unsigned char> m_imagePreScan;
  usImagePreScan2D<unsigned char> m_confidencePreScan;
  usImagePostScan2D<unsigned char> m_imagePostScan;

  bool m_trackerActivated;
  bool m_controllerActivated;
};
#endif // Qt & template tracking available
#endif // __usRectangleVisualServoingController_h_
