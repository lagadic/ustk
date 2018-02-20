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
 * @file usElastographyMainWindow.h
 * @brief Main window class for elastography display.
 */

#ifndef __usElastographyMainWindow_h_
#define __usElastographyMainWindow_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>
#include <QObject>
#include <QPixmap>
#include <visp3/io/vpImageIo.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF2D.h>
#include <visp3/ustk_gui/usElastographyQtWrapper.h>

/**
 * @class usElastographyMainWindow
 * @brief Main window class for elastography display.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usElastographyMainWindow : public QMainWindow
{
  Q_OBJECT
public:
  usElastographyMainWindow();
  ~usElastographyMainWindow();

public slots:
  void drawElasto(const vpImage<unsigned char> img);
  void setElastoROI(QRect rect);
  void updateFrame();

private:
  void convertVpImageToQImage();

  // Computing the strain map
  usElastographyQtWrapper m_elastographyWrapper;

  // Grabbing
  QThread *m_grabbingThread;
  usNetworkGrabberRF2D m_rfGrabber;

  // GUI
  QGraphicsScene *m_scene;
  QGraphicsView *m_view;
  QGraphicsPixmapItem m_item;

  // data
  vpImage<unsigned char> m_strainMap;
  QImage m_strainPixmap;
  unsigned char *bitmap;
  unsigned int m_bitmapSize;
};
#endif // QT && ELASTOGRAPHY
#endif // __usElastographyMainWindow_h_
