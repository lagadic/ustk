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
* @file usImageDisplayWidgetQmlOverlayServoing.cpp
* @brief Qt widget class for ultrasound image display.
* @warning This widget is based on QtQuick, and has been tested on qt 5.8 or newer versions.
*/

#include <visp3/ustk_gui/usImageDisplayWidgetQmlOverlayServoing.h>

#if (defined(USTK_HAVE_VTK_QT5) || defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT6))

#include <QQuickItem>

/**
* Constructor.
*/
usImageDisplayWidgetQmlOverlayServoing::usImageDisplayWidgetQmlOverlayServoing()
  : usImageDisplayWidget(), m_qQuickOverlay(), m_useFeatureDisplay(false), m_confidence(), m_plot(), m_startTime()
{
  this->setMinimumSize(200, 200);
  m_qQuickOverlay = new QQuickWidget(m_label);
  m_qQuickOverlay->setAttribute(Qt::WA_AlwaysStackOnTop);
  m_qQuickOverlay->setClearColor(Qt::transparent);
  m_qQuickOverlay->setSource(QUrl("qrc:/qml/overlay-servoing-rect-centering.qml"));

  connect(m_qQuickOverlay->rootObject(), SIGNAL(startTracking()), this, SLOT(startTrackingSlot()));
  connect(m_qQuickOverlay->rootObject(), SIGNAL(stopTracking()), this, SIGNAL(stopTrackingRect()));
  connect(m_qQuickOverlay->rootObject(), SIGNAL(startServoing()), this, SIGNAL(startServoingRect()));
  connect(m_qQuickOverlay->rootObject(), SIGNAL(stopServoing()), this, SIGNAL(stopServoingRect()));

  QQuickWidget::Status status(QQuickWidget::Null);

  while (status != QQuickWidget::Ready) {
    status = m_qQuickOverlay->status();
    vpTime::wait(50);
  }

#if defined(VISP_HAVE_GDI)
  m_display = new vpDisplayGDI;
#elif defined(VISP_HAVE_OPENCV)
  m_display = new vpDisplayOpenCV;
#endif
}

/**
* Destructor.
*/
usImageDisplayWidgetQmlOverlayServoing::~usImageDisplayWidgetQmlOverlayServoing() { }

void usImageDisplayWidgetQmlOverlayServoing::resizeEvent(QResizeEvent *event)
{
  event->accept();
  usImageDisplayWidget::resizeEvent(event);
  if (m_qQuickOverlay->rootObject()) {
    m_qQuickOverlay->rootObject()->setProperty("width", event->size().width());
    m_qQuickOverlay->rootObject()->setProperty("height", event->size().height());

    QObject *rectItem = m_qQuickOverlay->rootObject()->findChild<QObject *>("selectionRectangle");
    if (rectItem) {
      int x = rectItem->property("x").toInt() * event->size().height() / (double)size().height();
      int y = rectItem->property("y").toInt() * event->size().width() / (double)size().width();
      rectItem->setProperty("x", x);
      rectItem->setProperty("y", y);
    }
  }
}

/**
* Slot called to update the ultrasound image to display for pre-scan.
* @param img New ultrasound image to display.
*/
void usImageDisplayWidgetQmlOverlayServoing::updateFrame(const usImagePreScan2D<unsigned char> img)
{
  if (m_useScanConversion) {
    m_scanConverter.convert(img, m_postScan);
    m_QImage = QImage(m_postScan.bitmap, m_postScan.getWidth(), m_postScan.getHeight(), m_postScan.getWidth(),
                      QImage::Format_Indexed8);
  }
  else
    m_QImage = QImage(img.bitmap, img.getWidth(), img.getHeight(), img.getWidth(), QImage::Format_Indexed8);

  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();

  // update feature display
  if (m_useFeatureDisplay) {
#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
    if (m_confidence.getSize() > 0) {
      if (m_confidence.display == NULL)
        m_display->init(m_confidence);
      vpDisplay::display(m_confidence);
    }
#endif
  }
}

vpImagePoint usImageDisplayWidgetQmlOverlayServoing::displayImageToRealImageDimentions(const vpImagePoint displayPoint)
{
  vpImagePoint p;
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_useScanConversion) {
    imageHeight = m_postScan.getHeight();
    imageWidth = m_postScan.getWidth();
  }
  else {
    imageHeight = m_image.getHeight();
    imageWidth = m_image.getWidth();
  }
  p.set_i((displayPoint.get_i() * imageHeight) / (double)height());
  p.set_j((displayPoint.get_j() * imageWidth) / (double)width());

  return p;
}

vpImagePoint
usImageDisplayWidgetQmlOverlayServoing::realImageToDisplayImageDimentions(const vpImagePoint realImagePoint)
{
  vpImagePoint p;
  unsigned int imageHeight;
  unsigned int imageWidth;
  if (m_useScanConversion) {
    imageHeight = m_postScan.getHeight();
    imageWidth = m_postScan.getWidth();
  }
  else {
    imageHeight = m_image.getHeight();
    imageWidth = m_image.getWidth();
  }
  p.set_i((realImagePoint.get_i() * height()) / (double)imageHeight);
  p.set_j((realImagePoint.get_j() * width()) / (double)imageWidth);

  return p;
}

vpRectOriented
usImageDisplayWidgetQmlOverlayServoing::displayImageToRealImageDimentions(const vpRectOriented displayRectangle)
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
  }
  else {
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

vpRectOriented
usImageDisplayWidgetQmlOverlayServoing::realImageToDisplayImageDimentions(const vpRectOriented realRectangle)
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
  }
  else {
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
void usImageDisplayWidgetQmlOverlayServoing::updateRectPosition(vpRectOriented newRectangle)
{
  // transform rectangle from real image size to display dimentions
  vpRectOriented displayRectangle;
  displayRectangle = realImageToDisplayImageDimentions(newRectangle);

  QObject *rectItem = m_qQuickOverlay->rootObject()->findChild<QObject *>("selectionRectangle");
  rectItem->setProperty("x", displayRectangle.getCenter().get_j() - (displayRectangle.getWidth() / 2.0));
  rectItem->setProperty("y", displayRectangle.getCenter().get_i() - (displayRectangle.getHeight() / 2.0));
  rectItem->setProperty("rotation", vpMath::deg(displayRectangle.getOrientation()));
  rectItem->setProperty("height", displayRectangle.getHeight());
  rectItem->setProperty("width", displayRectangle.getWidth());
}

/**
* Start tacking slot. Gets the actual position of the displayed rectangle, and transmits it unsing startTracking signal
* (expressed in real image dimentions).
*/
void usImageDisplayWidgetQmlOverlayServoing::startTrackingSlot()
{
  QObject *rectangleObject = m_qQuickOverlay->rootObject()->findChild<QObject *>("selectionRectangle");
  int topLeftX = rectangleObject->property("x").toInt();
  int topLeftY = rectangleObject->property("y").toInt();
  int height = rectangleObject->property("height").toInt();
  int width = rectangleObject->property("width").toInt();
  int centerX = topLeftX + (width / 2.0);
  int centerY = topLeftY + (height / 2.0);
  int rotation = rectangleObject->property("rotation").toInt();

  vpRectOriented displayRect(vpImagePoint(centerY, centerX), width, height, vpMath::rad(rotation));

  emit(startTrackingRect(displayImageToRealImageDimentions(displayRect)));
}

void usImageDisplayWidgetQmlOverlayServoing::enableFeaturesDisplay()
{
  m_useFeatureDisplay = true;

  m_plot.init(2);
  m_plot.initGraph(0, 1);
  m_plot.setTitle(0, "confidence barycenter error");
  m_plot.setUnitY(0, "error");
  m_plot.setLegend(0, 0, "time");
  m_plot.initGraph(1, 1);
  m_plot.setTitle(1, "Lateral tracking error");
  m_plot.setUnitY(1, "error");
  m_plot.setLegend(1, 0, "time");
  m_startTime = vpTime::measureTimeMs();
}

void usImageDisplayWidgetQmlOverlayServoing::disableFeaturesDisplay() { m_useFeatureDisplay = false; }

void usImageDisplayWidgetQmlOverlayServoing::updateConfidenceAngle(double scanline)
{
#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  if (m_useFeatureDisplay) {
    vpImagePoint p0 = vpImagePoint(0, scanline);
    vpImagePoint p1 = vpImagePoint(m_confidence.getHeight() - 1, scanline);
    vpImagePoint p2 = vpImagePoint(0, m_confidence.getWidth() / 2);
    vpImagePoint p3 = vpImagePoint(m_confidence.getHeight() - 1, m_confidence.getWidth() / 2);
    vpDisplay::displayLine(m_confidence, p0, p1, vpColor::red);
    vpDisplay::displayLine(m_confidence, p2, p3, vpColor::green);
    vpDisplay::flush(m_confidence);

    // plot errors
    m_plot.plot(0, 0, vpTime::measureTimeMs() - m_startTime,
                vpMath::deg(scanline * m_confidence.getScanLinePitch() - m_confidence.getFieldOfView() / 2.0));
  }
#endif
}

void usImageDisplayWidgetQmlOverlayServoing::updateConfidenceMap(usImagePreScan2D<unsigned char> confidence)
{
  m_confidence = confidence;
}

void usImageDisplayWidgetQmlOverlayServoing::updateXError(double error)
{
#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  if (m_useFeatureDisplay) {
    // plot errors
    m_plot.plot(1, 0, vpTime::measureTimeMs() - m_startTime, error);
  }
#endif
}

#endif
