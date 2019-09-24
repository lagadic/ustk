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
* @file usElastographyDisplayWidget.cpp
* @brief Qt widget class for ultrasound image display.
*/

#include <visp3/ustk_gui/usElastographyDisplayWidget.h>
#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

/**
* Constructor.
*/
usElastographyDisplayWidget::usElastographyDisplayWidget()
  : m_label(NULL), m_QImage(), m_pixmap(), m_rgbBitmap(NULL), m_bitmapSize(0)
{
  this->setMinimumSize(50, 50);
  m_label = new QLabel(this);
  m_label->setMinimumSize(50, 50);
  m_label->autoFillBackground();
}

/**
* Destructor.
*/
usElastographyDisplayWidget::~usElastographyDisplayWidget() { delete m_label; }

/**
* Slot called to update the ultrasound image to display.
* @param elastographyImage New elastography image to display.
*/
void usElastographyDisplayWidget::updateFrame(const vpImage<vpRGBa> elastographyImage)
{
  if (m_bitmapSize != elastographyImage.getSize() * 3) {
    m_rgbBitmap = new uchar[elastographyImage.getSize() * 3];
    m_bitmapSize = elastographyImage.getSize() * 3;
  }
// loop on every pixel
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)elastographyImage.getSize(); i++) {
    m_rgbBitmap[i * 3] = elastographyImage.bitmap[i].R;
    m_rgbBitmap[i * 3 + 1] = elastographyImage.bitmap[i].G;
    m_rgbBitmap[i * 3 + 2] = elastographyImage.bitmap[i].B;
  }
  m_QImage = QImage(m_rgbBitmap, elastographyImage.getWidth(), elastographyImage.getHeight(),
                    elastographyImage.getWidth() * 3, QImage::Format_RGB888);
  // m_QImage.save("qimage.png");
  QImage I = m_QImage.scaled(this->width(), this->height());
  // I.save("qimageScaled.png");
  m_pixmap = QPixmap::fromImage(I);
  m_pixmap.save("qpixmap.png");
  m_label->setPixmap(m_pixmap);
  m_label->update();
}

void usElastographyDisplayWidget::resizeEvent(QResizeEvent *event)
{
  m_label->resize(event->size());
  QImage I = m_QImage.scaled(this->width(), this->height());
  m_pixmap = QPixmap::fromImage(I);
  m_label->setPixmap(m_pixmap);
  m_label->update();
}

#endif
