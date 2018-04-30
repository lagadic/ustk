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
* @file usImageDisplayWidget.cpp
* @brief Qt widget class for ultrasound image display.
*/

#include <visp3/ustk_gui/usImageDisplayWidget.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5))

/**
* Constructor.
*/
usImageDisplayWidget::usImageDisplayWidget() : m_label(NULL), m_QImage(), m_pixmap(),m_displayPostScan(true),
  m_scanConverter(), m_postScan(), m_leftArrow(),m_rightArrow(), m_confidenceServoingButton()
{
  this->setMinimumSize(200, 200);
  m_label = new QLabel(this);
  m_label->setMinimumSize(200, 200);
  m_label->autoFillBackground();

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

  connect(&m_leftArrow,SIGNAL(pressed()),this,SIGNAL(moveLeft()));
  connect(&m_rightArrow,SIGNAL(pressed()),this,SIGNAL(moveRight()));
  connect(&m_leftArrow,SIGNAL(released()),this,SIGNAL(stopMove()));
  connect(&m_rightArrow,SIGNAL(released()),this,SIGNAL(stopMove()));

  connect(&m_confidenceServoingButton,SIGNAL(toggled(bool)),this,SIGNAL(confidenceServoing(bool)));
}

/**
* Destructor.
*/
usImageDisplayWidget::~usImageDisplayWidget() {
  if(m_label)
    delete m_label;
}

/**
* Slot called to update the ultrasound image to display.
* @param img New ultrasound image to display.
*/
void usImageDisplayWidget::updateFrame(const vpImage<unsigned char> img)
{
  m_QImage = QImage(img.bitmap, img.getWidth(), img.getHeight(), img.getWidth(), QImage::Format_Indexed8);
  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();

  if(m_controlArrowsActivated)
    enableControlArrows();
}

/**
* Slot called to update the ultrasound image to display for pre-scan.
* @param img New ultrasound image to display.
*/
void usImageDisplayWidget::updateFrame(const usImagePreScan2D<unsigned char> img)
{
  if(m_displayPostScan) {
    m_scanConverter.convert(img, m_postScan);
    m_QImage = QImage(m_postScan.bitmap, m_postScan.getWidth(), m_postScan.getHeight(), m_postScan.getWidth(), QImage::Format_Indexed8);
  }
  else
    m_QImage = QImage(img.bitmap, img.getWidth(), img.getHeight(), img.getWidth(), QImage::Format_Indexed8);

  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();

  if(m_controlArrowsActivated)
    enableControlArrows();
}

void usImageDisplayWidget::resizeEvent(QResizeEvent *event)
{
  m_label->resize(event->size());
  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();

  m_confidenceServoingButton.setGeometry(this->width()/2,10,40,40);
  m_confidenceServoingButton.raise();

  if(m_controlArrowsActivated)
    enableControlArrows();
}

void usImageDisplayWidget::enableControlArrows() {
  m_controlArrowsActivated = true;

  m_leftArrow.setVisible(true);
  m_rightArrow.setVisible(true);
  m_leftArrow.setGeometry(10,this->height()/2,40,40);
  m_rightArrow.setGeometry(this->width()-50,this->height()/2,40,40);
  m_leftArrow.raise();
  m_rightArrow.raise();
}

void usImageDisplayWidget::disableControlArrows() {
  m_controlArrowsActivated = false;
  m_leftArrow.hide();
  m_rightArrow.hide();
}


void usImageDisplayWidget::enablePostScanDisplay(bool enable) {
  m_displayPostScan = enable;
}


#endif
