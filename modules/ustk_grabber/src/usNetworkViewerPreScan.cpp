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

#include <visp3/ustk_grabber/usNetworkViewerPreScan.h>

#if defined(USTK_GRABBER_HAVE_QT5) && defined(VISP_HAVE_DISPLAY)

/**
* Constructor. Inititializes the image, and the vpDisplay.
*/
usNetworkViewerPreScan::usNetworkViewerPreScan(int frameWith, int frameHeight, QObject *parent) :
  QObject(parent)
{
  m_grabbedImage.init(frameWith,frameHeight);
  display.init(m_grabbedImage);
}

/**
* Destructor.
*/
usNetworkViewerPreScan::~usNetworkViewerPreScan()
{

}

/**
* Slot to copy the image, and update the display, called when a new frame is available on the network.
*/
void usNetworkViewerPreScan::updateDisplay(usImagePreScan2D<unsigned char>* newFrame) {
  m_grabbedImage.resize(newFrame->getHeight(),newFrame->getWidth());
  if(display.getHeight() != m_grabbedImage.getHeight() || display.getWidth() != m_grabbedImage.getWidth()) {
    display.close(m_grabbedImage);
    display.init(m_grabbedImage);
  }
  memcpy(m_grabbedImage.bitmap,newFrame->bitmap,newFrame->getSize());

  vpDisplay::display(m_grabbedImage);
  vpDisplay::flush(m_grabbedImage);
}

#endif
