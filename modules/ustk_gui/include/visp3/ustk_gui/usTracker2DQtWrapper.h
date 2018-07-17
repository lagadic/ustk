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
 * @file usTracker2DQtWrapper.h
 * @brief Qt wrapper for usDenseTracker2D class.
 */

#ifndef __usTracker2DQtWrapper_h_
#define __usTracker2DQtWrapper_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_TEMPLATE_TRACKING)

#include <QObject>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_template_tracking/usDenseTracker2D.h>

/**
 * @class usTracker2DQtWrapper
 * @brief Qt wrapper for usDenseTracker2D class.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usTracker2DQtWrapper : public QObject
{
  Q_OBJECT
public:
  usTracker2DQtWrapper();
  ~usTracker2DQtWrapper();

public slots:
  void initTracker(vpRectOriented rect);
  void updateImage(vpImage<unsigned char> image);
  void updateImage(usImagePreScan2D<unsigned char> image);
  void updateImage(usImagePostScan2D<unsigned char> image);
  void stopTracking();

signals:
  void newTrackedRectangle(vpRectOriented);

private:
  usDenseTracker2D m_tracker;
  vpImage<unsigned char> m_firstImage;
  bool m_firstFrameArrived;
  bool m_isInitialized;
};
#endif // QT && Template tracking module
#endif // __usTracker2DQtWrapper_h_
