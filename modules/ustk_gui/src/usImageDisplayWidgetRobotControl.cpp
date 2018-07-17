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
* @file usImageDisplayWidgetRobotControl.cpp
* @brief Qt widget class for ultrasound image display.
*/

#include <visp3/ustk_gui/usImageDisplayWidgetRobotControl.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5))

/**
* Constructor.
*/
usImageDisplayWidgetRobotControl::usImageDisplayWidgetRobotControl()
  : usImageDisplayWidget(), m_controlArrowsActivated(false), m_leftArrow(), m_rightArrow(),
    m_confidenceServoingButton(), m_useFeatureDisplay(false), m_confidence(), m_plot(), m_startTime()
{
  this->setMinimumSize(200, 200);

  m_controlArrowsActivated = false;
  m_leftArrow.setParent(this);
  m_rightArrow.setParent(this);
  m_confidenceServoingButton.setParent(this);

  m_leftArrow.setText(QString("\u25C0"));
  m_rightArrow.setText(QString("\u25B6"));
  m_leftArrow.setVisible(false);
  m_rightArrow.setVisible(false);

  m_confidenceServoingButton.setText(QString("\u21BB"));
  m_confidenceServoingButton.setVisible(true);
  m_confidenceServoingButton.setCheckable(true);
  m_confidenceServoingButton.setChecked(false);
  m_confidenceServoingButton.setAutoFillBackground(true);
  QPalette pal = m_confidenceServoingButton.palette();
  pal.setColor(QPalette::Button, QColor(Qt::green));
  m_confidenceServoingButton.setPalette(pal);
  m_confidenceServoingButton.update();

#if defined(VISP_HAVE_GDI)
  m_display = new vpDisplayGDI;
#elif defined(VISP_HAVE_OPENCV)
  m_display = new vpDisplayOpenCV;
#endif

  connect(&m_leftArrow, SIGNAL(pressed()), this, SIGNAL(moveLeft()));
  connect(&m_rightArrow, SIGNAL(pressed()), this, SIGNAL(moveRight()));
  connect(&m_leftArrow, SIGNAL(released()), this, SIGNAL(stopMove()));
  connect(&m_rightArrow, SIGNAL(released()), this, SIGNAL(stopMove()));

  connect(&m_confidenceServoingButton, SIGNAL(toggled(bool)), this, SLOT(updateConfidenceServoingStatus(bool)));
}

/**
* Destructor.
*/
usImageDisplayWidgetRobotControl::~usImageDisplayWidgetRobotControl() {}

/**
* Slot called to update the ultrasound image to display.
* @param img New ultrasound image to display.
*/
void usImageDisplayWidgetRobotControl::updateFrame(const vpImage<unsigned char> img)
{
  m_QImage = QImage(img.bitmap, img.getWidth(), img.getHeight(), img.getWidth(), QImage::Format_Indexed8);
  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();

  if (m_controlArrowsActivated)
    enableControlArrows();
}

/**
* Slot called to update the ultrasound image to display for pre-scan.
* @param img New ultrasound image to display.
*/
void usImageDisplayWidgetRobotControl::updateFrame(const usImagePreScan2D<unsigned char> img)
{
  if (m_useScanConversion) {
    m_scanConverter.convert(img, m_postScan);
    m_QImage = QImage(m_postScan.bitmap, m_postScan.getWidth(), m_postScan.getHeight(), m_postScan.getWidth(),
                      QImage::Format_Indexed8);
  } else
    m_QImage = QImage(img.bitmap, img.getWidth(), img.getHeight(), img.getWidth(), QImage::Format_Indexed8);

  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();

  if (m_controlArrowsActivated)
    enableControlArrows();

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

void usImageDisplayWidgetRobotControl::resizeEvent(QResizeEvent *event)
{
  m_label->resize(event->size());
  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();

  m_confidenceServoingButton.setGeometry((this->width() / 2) - m_confidenceServoingButton.size().width() / 2, 10, 40,
                                         40);
  m_confidenceServoingButton.raise();

  if (m_controlArrowsActivated)
    enableControlArrows();
}

void usImageDisplayWidgetRobotControl::enableControlArrows()
{
  m_controlArrowsActivated = true;

  m_leftArrow.setVisible(true);
  m_rightArrow.setVisible(true);
  m_leftArrow.setGeometry(10, (this->height() / 2) - m_leftArrow.size().height() / 2, 40, 40);
  m_rightArrow.setGeometry(this->width() - 50, (this->height() / 2) - m_rightArrow.size().height() / 2, 40, 40);
  m_leftArrow.raise();
  m_rightArrow.raise();
}

void usImageDisplayWidgetRobotControl::disableControlArrows()
{
  m_controlArrowsActivated = false;
  m_leftArrow.hide();
  m_rightArrow.hide();
}

void usImageDisplayWidgetRobotControl::updateConfidenceServoingStatus(bool activate)
{
  if (activate) {
    QPalette pal = m_confidenceServoingButton.palette();
    pal.setColor(QPalette::Button, QColor(Qt::red));
    m_confidenceServoingButton.setPalette(pal);
    m_confidenceServoingButton.update();
  } else {
    QPalette pal = m_confidenceServoingButton.palette();
    pal.setColor(QPalette::Button, QColor(Qt::green));
    m_confidenceServoingButton.setPalette(pal);
    m_confidenceServoingButton.update();
  }

  emit(confidenceServoing(activate));
}

void usImageDisplayWidgetRobotControl::enableFeaturesDisplay()
{
  m_useFeatureDisplay = true;

  m_plot.init(1);
  m_plot.initGraph(0, 1);
  m_plot.setTitle(0, "confidence barycenter error");
  m_plot.setUnitY(0, "error");
  m_plot.setLegend(0, 0, "time");
  m_startTime = vpTime::measureTimeMs();
}

void usImageDisplayWidgetRobotControl::disableFeaturesDisplay() { m_useFeatureDisplay = false; }

void usImageDisplayWidgetRobotControl::updateConfidenceAngle(double sanline)
{
#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  if (m_useFeatureDisplay) {
    vpImagePoint p0 = vpImagePoint(0, sanline);
    vpImagePoint p1 = vpImagePoint(m_confidence.getHeight() - 1, sanline);
    vpImagePoint p2 = vpImagePoint(0, m_confidence.getWidth() / 2);
    vpImagePoint p3 = vpImagePoint(m_confidence.getHeight() - 1, m_confidence.getWidth() / 2);
    vpDisplay::displayLine(m_confidence, p0, p1, vpColor::red);
    vpDisplay::displayLine(m_confidence, p2, p3, vpColor::green);
    vpDisplay::flush(m_confidence);

    // plot errors
    m_plot.plot(0, 0, vpTime::measureTimeMs() - m_startTime,
                vpMath::deg(sanline * m_confidence.getScanLinePitch() - m_confidence.getFieldOfView() / 2.0));
  }
#endif
}

void usImageDisplayWidgetRobotControl::updateConfidenceMap(usImagePreScan2D<unsigned char> confidence)
{
  m_confidence = confidence;
}
#endif
