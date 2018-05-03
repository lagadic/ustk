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
* @file usImageDisplayWidgetQmlOverlay.cpp
* @brief Qt widget class for ultrasound image display.
*/

#include <QQuickItem>
#include <visp3/ustk_gui/usImageDisplayWidgetQmlOverlay.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5))

/**
* Constructor.
*/
usImageDisplayWidgetQmlOverlay::usImageDisplayWidgetQmlOverlay() : usImageDisplayWidget(), m_qQuickOverlay()
{
  this->setMinimumSize(200, 200);
  m_qQuickOverlay = new QQuickWidget(m_label);
  m_qQuickOverlay->setAttribute(Qt::WA_AlwaysStackOnTop);
  m_qQuickOverlay->setClearColor(Qt::transparent);
  m_qQuickOverlay->setSource(QUrl::fromLocalFile("overlay.qml"));

  connect(m_qQuickOverlay->rootObject(), SIGNAL(startTracking()), this, SIGNAL(startTracking()));
  connect(m_qQuickOverlay->rootObject(), SIGNAL(stopTracking()), this, SIGNAL(stopTracking()));
}

/**
* Destructor.
*/
usImageDisplayWidgetQmlOverlay::~usImageDisplayWidgetQmlOverlay() {}

void usImageDisplayWidgetQmlOverlay::resizeEvent(QResizeEvent *event)
{
  m_label->resize(event->size());

  m_qQuickOverlay->rootObject()->setProperty("width", event->size().width());
  m_qQuickOverlay->rootObject()->setProperty("height", event->size().height());

  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();
}

vpImagePoint usImageDisplayWidgetQmlOverlay::toRealImageDimentions(const vpImagePoint displayPoint)
{
  vpImagePoint p;
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_displayPostScan) {
    imageHeight = m_postScan.getHeight();
    imageWidth = m_postScan.getWidth();
  } else {
    imageHeight = m_image.getHeight();
    imageWidth = m_image.getWidth();
  }
  p.set_i((displayPoint.get_i() * imageHeight) / (double)height());
  p.set_j((displayPoint.get_j() * imageWidth) / (double)width());

  return p;
}

vpImagePoint usImageDisplayWidgetQmlOverlay::fromRealImageDimentions(vpImagePoint realImagePoint)
{
  vpImagePoint p;
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_displayPostScan) {
    imageHeight = m_postScan.getHeight();
    imageWidth = m_postScan.getWidth();
  } else {
    imageHeight = m_image.getHeight();
    imageWidth = m_image.getWidth();
  }
  p.set_i((realImagePoint.get_i() * height()) / (double)imageHeight);
  p.set_j((realImagePoint.get_j() * width()) / (double)imageWidth);

  return p;
}

void usImageDisplayWidgetQmlOverlay::updateRectPosition(vpImagePoint centerCoordsInRealImage, double theta)
{
  QObject *rectItem = m_qQuickOverlay->rootObject()->findChild<QObject *>("selectionRectangle");
  vpImagePoint newCoord = fromRealImageDimentions(centerCoordsInRealImage);
  rectItem->setProperty("x", newCoord.get_i());
  rectItem->setProperty("y", newCoord.get_j());
}

void usImageDisplayWidgetQmlOverlay::updateRectPosition(vpRectOriented newRectangle)
{
  updateRectPosition(newRectangle.getCenter(), newRectangle.getOrientation());
}

#endif
