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
* @file usElastographyDisplayWidget.h
* @brief QWidget-based class for elastography display.
*/

#ifndef __usElastographyDisplayWidget_h_
#define __usElastographyDisplayWidget_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

#include <QImage>
#include <QLabel>
#include <QObject>
#include <QPixmap>
#include <QResizeEvent>
#include <QWidget>
#include <visp3/ustk_elastography/usImageElastography.h>

/**
 * @class usElastographyDisplayWidget
 * @brief QWidget-based class for elastography display.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usElastographyDisplayWidget : public QWidget
{
  Q_OBJECT
public:
  usElastographyDisplayWidget();
  ~usElastographyDisplayWidget();

  void resizeEvent(QResizeEvent *event);

public slots:
  void updateFrame(const vpImage<vpRGBa> elastographyImage);

private:
  QLabel *m_label;

  // data
  QImage m_QImage;
  QPixmap m_pixmap;

  // bitmap, for RGBa -> aRGB conversion
  uchar *m_rgbBitmap;
  unsigned int m_bitmapSize;
};
#endif // QT && ELASTOGRAPHY
#endif // __usElastographyDisplayWidget_h_
