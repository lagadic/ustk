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
 * @file usConfidenceMapController.h
 * @brief Wrapper aroud usScanlineConfidence2D to control a robot using with Qt sigals/slots mecanism.
 */

#ifndef __uusConfidenceMapController_h_
#define __uusConfidenceMapController_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_CONFIDENCE_MAP)

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>

#include <QObject>

/**
 * @class usConfidenceMapController
 * @brief Wrapper aroud usScanlineConfidence2D to control a robot using with Qt sigals/slots mecanism.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usConfidenceMapController : public QObject
{
  Q_OBJECT
public:
  usConfidenceMapController(QObject *parent = NULL);
  virtual ~usConfidenceMapController();

  void setPropotionnalControlGain(const double gain);
  double getPropotionnalControlGain() const;

signals:
  void updateProbeOrientation(int angularVelocity);

public slots:
  void updateImage(usImagePreScan2D<unsigned char> image);
  void activateController(bool activate);

private:
  usScanlineConfidence2D m_confidenceProcessor;
  usImagePreScan2D<unsigned char> m_confidenceMap;

  double m_gain;
  bool m_activated;
};
#endif
#endif // US_VIEWER_WIDGET
