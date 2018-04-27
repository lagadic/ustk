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
usImageDisplayWidget::usImageDisplayWidget() : m_label(NULL), m_QImage(), m_pixmap(),m_leftArrow(),m_rightArrow()
{
  this->setMinimumSize(50, 50);
  m_label = new QLabel(this);
  m_label->setMinimumSize(50, 50);
  m_label->autoFillBackground();

  m_controlArrowsActivated = false;
  m_leftArrow.setParent(m_label);
  m_rightArrow.setParent(m_label);

  m_leftArrow.setText(QString("\u25C0"));
  m_rightArrow.setText(QString("\u25B6"));
  m_leftArrow.setVisible(false);
  m_rightArrow.setVisible(false);

  connect(&m_leftArrow,SIGNAL(pressed()),this,SIGNAL(moveLeft()));
  connect(&m_rightArrow,SIGNAL(pressed()),this,SIGNAL(moveRight()));
  connect(&m_leftArrow,SIGNAL(released()),this,SIGNAL(stopMove()));
  connect(&m_rightArrow,SIGNAL(released()),this,SIGNAL(stopMove()));
}

/**
* Destructor.
*/
usImageDisplayWidget::~usImageDisplayWidget() { delete m_label; }

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
  if(m_controlArrowsActivated)
    enableControlArrows();

}

void usImageDisplayWidget::enableControlArrows() {
  m_controlArrowsActivated = true;

  m_leftArrow.setVisible(true);
  m_rightArrow.setVisible(true);
  m_leftArrow.setGeometry(10,m_label->height()/2,40,40);
  m_rightArrow.setGeometry(m_label->width()-50,m_label->height()/2,40,40);
  m_leftArrow.raise();
  m_rightArrow.raise();
}

void usImageDisplayWidget::disableControlArrows() {
  m_controlArrowsActivated = false;
  m_leftArrow.hide();
  m_rightArrow.hide();
}

#endif
