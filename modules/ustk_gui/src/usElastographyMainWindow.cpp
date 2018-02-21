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
* @file usElastographyMainWindow.cpp
* @brief Qt wrapper for elastography computation.
*/

#include <visp3/ustk_gui/usElastographyMainWindow.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

/**
* Constructor.
*/
usElastographyMainWindow::usElastographyMainWindow()
  : m_elastographyWrapper(), m_grabbingThread(NULL), m_rfGrabber(), m_scene(NULL), m_view(NULL), m_item(),
    m_strainMap(), m_strainPixmap(), m_bitmapSize(0)
{
  m_scene = new QGraphicsScene(0, 0, 200, 200, this);
  m_view = new QGraphicsView(m_scene, this);
  this->setCentralWidget(m_view);

  m_item.setFlag(QGraphicsItem::ItemIsMovable, false);
  m_item.setFlag(QGraphicsItem::ItemIsSelectable, false);
  m_item.setFlag(QGraphicsItem::ItemIsFocusable, false);

  connect(&m_elastographyWrapper, SIGNAL(elastoReady(vpImage<unsigned char>)), this,
          SLOT(drawElasto(const vpImage<unsigned char>)));

  connect(&m_rfGrabber, SIGNAL(newFrameAvailable()), this, SLOT(updateFrame()));

  // grabber
  m_grabbingThread = new QThread();
  m_rfGrabber.setIPAddress("127.0.0.1");
  m_rfGrabber.connectToServer();

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;     // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0, RF = 12

  // sending acquisition parameters
  m_rfGrabber.initAcquisition(header);

  m_rfGrabber.runAcquisition();

  // Move the grabber object to another thread
  m_rfGrabber.moveToThread(m_grabbingThread);
  m_grabbingThread->start();
}

/**
* Destructor.
*/
usElastographyMainWindow::~usElastographyMainWindow()
{
  delete m_scene;
  delete m_view;
  delete m_grabbingThread;
}

/**
* Slot called to update the RF frame to compute a new strain map.
* @param img New RF frame to use for elastography.
*/
void usElastographyMainWindow::updateFrame()
{
  usFrameGrabbedInfo<usImageRF2D<short int> > *grabbedFrame = m_rfGrabber.acquire();
  m_elastographyWrapper.updateFrame(*grabbedFrame);
}

/**
* Slot called to update the ROI of the elastography
* @param rect The new ROI to perform elastography on.
*/
void usElastographyMainWindow::setElastoROI(QRect rect)
{
  m_scene->setSceneRect(10, 10, rect.width(), rect.height());
  m_elastographyWrapper.setROI(40, 2500, 50, 500);
}

/**
* Slot to update the RF frame as input for elastography. The elastography is computed between the previous frame and the
* new one.
* @param img New RF image input for elastography computation.
*/
void usElastographyMainWindow::drawElasto(const vpImage<unsigned char> img)
{
  m_strainMap = img;
  convertVpImageToQImage();
  m_item.setPixmap(QPixmap::fromImage(m_strainPixmap));
  if (m_scene->items().size() == 0)
    m_scene->addItem(&m_item);
  m_scene->update(m_scene->itemsBoundingRect());
}

void usElastographyMainWindow::convertVpImageToQImage()
{
  if (m_bitmapSize != m_strainMap.getHeight() * m_strainMap.getWidth()) {
    bitmap = new unsigned char[m_strainMap.getHeight() * m_strainMap.getWidth()];
    m_bitmapSize = m_strainMap.getHeight() * m_strainMap.getWidth();
  }
  memcpy(bitmap, m_strainMap.bitmap, m_strainMap.getHeight() * m_strainMap.getWidth());

  m_strainPixmap = QImage((uchar *)bitmap, m_strainMap.getWidth(), m_strainMap.getHeight(), m_strainMap.getWidth(),
                          QImage::Format_Grayscale8);
}

#endif
