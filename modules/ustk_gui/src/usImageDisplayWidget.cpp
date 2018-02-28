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
usImageDisplayWidget::usImageDisplayWidget() : m_label(NULL), m_QImage()
{
  this->setMinimumSize(50, 50);
  m_label = new QLabel(this);
  m_label->setMinimumSize(50, 50);
  m_label->autoFillBackground();
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
}

void usImageDisplayWidget::resizeEvent(QResizeEvent *event)
{
  m_label->resize(event->size());
  QImage I = m_QImage.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();
}

#endif
