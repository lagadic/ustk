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
 * @file usImageDisplayWidgetQmlOverlayServoing.h
 * @brief Qt widget class for 2D ultrasound image display, containing robot control tools for visual servoing on a rectangular ROI.
 */

#ifndef __usImageDisplayWidgetQmlOverlayServoing_h_
#define __usImageDisplayWidgetQmlOverlayServoing_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5))

#include <QPushButton>
#include <QQuickWidget>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRectOriented.h>
#include <visp3/ustk_gui/usImageDisplayWidget.h>

/**
 * @class usImageDisplayWidgetQmlOverlayServoing
 * @brief Qt widget class for 2D ultrasound image display, containing robot control tools for visual servoing on a rectangular ROI.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usImageDisplayWidgetQmlOverlayServoing : public usImageDisplayWidget
{
  Q_OBJECT
public:
  usImageDisplayWidgetQmlOverlayServoing();
  ~usImageDisplayWidgetQmlOverlayServoing();

  void resizeEvent(QResizeEvent *event);

public slots:
  void updateRectPosition(vpRectOriented newRectangle);
  void startTrackingSlot();

signals:
  void startTrackingRect(vpRectOriented);
  void stopTrackingRect();
  void startServoingRect();
  void stopServoingRect();

private:
  vpImagePoint displayImageToRealImageDimentions(const vpImagePoint displayPoint);
  vpImagePoint realImageToDisplayImageDimentions(const vpImagePoint realImagePoint);

  vpRectOriented displayImageToRealImageDimentions(const vpRectOriented displayRectangle);
  vpRectOriented realImageToDisplayImageDimentions(const vpRectOriented realRectangle);

  QQuickWidget *m_qQuickOverlay;
  bool m_isTrackingRect;
  bool m_isServoingRect;
  bool m_isServoingConfidence;
};
#endif // QT && ELASTOGRAPHY
#endif // __usImageDisplayWidgetQmlOverlayServoing_h_