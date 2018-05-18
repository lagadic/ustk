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
* @warning This widget is based on QtQuick, and has been tested on qt 5.8 or newer versions.
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
  m_qQuickOverlay->setSource(QUrl("qrc:/qml/overlay.qml"));

  connect(m_qQuickOverlay->rootObject(), SIGNAL(startTracking()), this, SLOT(startTrackingSlot()));
  connect(m_qQuickOverlay->rootObject(), SIGNAL(stopTracking()), this, SIGNAL(stopTracking()));

  QQuickWidget::Status status(QQuickWidget::Null);

  while(status!=QQuickWidget::Ready) {
    status = m_qQuickOverlay->status();
    vpTime::wait(50);
  }
}

/**
* Destructor.
*/
usImageDisplayWidgetQmlOverlay::~usImageDisplayWidgetQmlOverlay() {}

void usImageDisplayWidgetQmlOverlay::resizeEvent(QResizeEvent *event)
{
  event->accept();
  usImageDisplayWidget::resizeEvent(event);
  if( m_qQuickOverlay->rootObject()) {
    m_qQuickOverlay->rootObject()->setProperty("width", event->size().width());
    m_qQuickOverlay->rootObject()->setProperty("height", event->size().height());
  }
}

vpImagePoint usImageDisplayWidgetQmlOverlay::displayImageToRealImageDimentions(const vpImagePoint displayPoint)
{
  vpImagePoint p;
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_useScanConversion) {
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

vpImagePoint usImageDisplayWidgetQmlOverlay::realImageToDisplayImageDimentions(const vpImagePoint realImagePoint)
{
  vpImagePoint p;
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_useScanConversion) {
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

vpRectOriented usImageDisplayWidgetQmlOverlay::displayImageToRealImageDimentions(const vpRectOriented displayRectangle)
{
  // center point
  vpImagePoint center;
  center = displayImageToRealImageDimentions(displayRectangle.getCenter());

  // rectangle height & width scaling using angular parameter
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_useScanConversion) {
    imageHeight = m_postScan.getHeight();
    imageWidth = m_postScan.getWidth();
  } else {
    imageHeight = m_image.getHeight();
    imageWidth = m_image.getWidth();
  }
  int newHeight =
      (displayRectangle.getHeight() * imageHeight / (double)height()) * std::cos(displayRectangle.getOrientation()) +
      (displayRectangle.getWidth() * imageWidth / (double)width()) * std::sin(displayRectangle.getOrientation());
  int newWidth =
      (displayRectangle.getHeight() * imageHeight / (double)height()) * std::sin(displayRectangle.getOrientation()) +
      (displayRectangle.getWidth() * imageWidth / (double)width()) * std::cos(displayRectangle.getOrientation());

  return vpRectOriented(center, newWidth, newHeight, displayRectangle.getOrientation());
}

vpRectOriented usImageDisplayWidgetQmlOverlay::realImageToDisplayImageDimentions(const vpRectOriented realRectangle)
{
  // center point
  vpImagePoint center;
  center = realImageToDisplayImageDimentions(realRectangle.getCenter());

  // rectangle height & width scaling using angular parameter
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_useScanConversion) {
    imageHeight = m_postScan.getHeight();
    imageWidth = m_postScan.getWidth();
  } else {
    imageHeight = m_image.getHeight();
    imageWidth = m_image.getWidth();
  }
  int newHeight =
      (realRectangle.getHeight() * height() / (double)imageHeight) * std::cos(realRectangle.getOrientation()) +
      (realRectangle.getWidth() * width() / (double)imageWidth) * std::sin(realRectangle.getOrientation());

  int newWidth =
      (realRectangle.getHeight() * height() / (double)imageHeight) * std::sin(realRectangle.getOrientation()) +
      (realRectangle.getWidth() * width() / (double)imageWidth) * std::cos(realRectangle.getOrientation());

  return vpRectOriented(center, newWidth, newHeight, realRectangle.getOrientation());
}

/**
* Update rectangle position slot.
* \param newRectangle New rectangle, expressed in real image dimensions.
*/
void usImageDisplayWidgetQmlOverlay::updateRectPosition(vpRectOriented newRectangle)
{
  // transform rectangle from real image size to display dimentions
  vpRectOriented displayRectangle;
  displayRectangle = realImageToDisplayImageDimentions(newRectangle);

  QObject *rectItem = m_qQuickOverlay->rootObject()->findChild<QObject *>("selectionRectangle");
  rectItem->setProperty("x", displayRectangle.getCenter().get_j());
  rectItem->setProperty("y", displayRectangle.getCenter().get_i());
  rectItem->setProperty("rotation", vpMath::deg(displayRectangle.getOrientation()));
  rectItem->setProperty("height", displayRectangle.getHeight());
  rectItem->setProperty("width", displayRectangle.getWidth());
}

/**
* Start tacking slot. Gets the actual position of the displayed rectangle, and transmits it unsing startTracking signal.
*/
void usImageDisplayWidgetQmlOverlay::startTrackingSlot()
{
  QObject *rectangleObject = m_qQuickOverlay->rootObject()->findChild<QObject *>("selectionRectangle");
  int centerX = rectangleObject->property("x").toInt();
  int centerY = rectangleObject->property("y").toInt();
  int height = rectangleObject->property("height").toInt();
  int width = rectangleObject->property("width").toInt();
  int rotation = rectangleObject->property("rotation").toInt();

  vpRectOriented displayRect(vpImagePoint(centerY, centerX), width, height, rotation);

  emit(startTracking(displayImageToRealImageDimentions(displayRect)));
}

#endif
